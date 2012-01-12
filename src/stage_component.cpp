/*
 *  stageros
 *  Copyright (c) 2008, Willow Garage, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/**

@mainpage

@htmlinclude manifest.html
**/

#define ROS_ERROR(x) std::cout << x << std::endl
#define ROS_DEBUG(x) std::cout << x << std::endl
#define ROS_WARN(x) std::cout << x << std::endl

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <signal.h>


// libstage
#include <stage.hh>

// roscpp
#include <darc/component.h>
#include <darc/pubsub/publisher.h>
#include <darc/pubsub/subscriber.h>
#include <darc/timer/periodic_timer.h>

#include <tf2_darc/transform_broadcaster.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <rosgraph_msgs/Clock.h>

#include <LinearMath/btQuaternion.h>
#include <LinearMath/btTransform.h>

//#include "tf/transform_broadcaster.h"

#define USAGE "stageros <worldfile>"
#define ODOM "odom"
#define BASE_SCAN "base_scan"
#define BASE_POSE_GROUND_TRUTH "base_pose_ground_truth"
#define CMD_VEL "cmd_vel"

// Our node
class StageComponent : public darc::Component
{
  private:
    // Messages that we'll send or receive
    sensor_msgs::LaserScan *laserMsgs;
    nav_msgs::Odometry *odomMsgs;
    nav_msgs::Odometry *groundTruthMsgs;
    rosgraph_msgs::Clock clockMsg;

    // A mutex to lock access to fields that are used in message callbacks
    boost::mutex msg_lock;

    // The models that we're interested in
    std::vector<Stg::ModelRanger *> lasermodels;
    std::vector<Stg::ModelPosition *> positionmodels;
  std::vector<darc::pubsub::Publisher<sensor_msgs::LaserScan>*> laser_pubs_;
  std::vector<darc::pubsub::Publisher<nav_msgs::Odometry>*> odom_pubs_;
  std::vector<darc::pubsub::Publisher<nav_msgs::Odometry>*> ground_truth_pubs_;
  std::vector<darc::pubsub::Subscriber<geometry_msgs::Twist>*> cmdvel_subs_;

  darc::timer::PeriodicTimer timer_;
  //    ros::Publisher clock_pub_;

    // A helper function that is executed for each stage model.  We use it
    // to search for models of interest.
    static void ghfunc(Stg::Model* mod, StageComponent* node);

    static bool s_update(Stg::World* world, StageComponent* node)
    {
      node->WorldCallback();
      // We return false to indicate that we want to be called again (an
      // odd convention, but that's the way that Stage works).
      return false;
    }

    // Appends the given robot ID to the given message name.  If omitRobotID
    // is true, an unaltered copy of the name is returned.
    const char *mapName(const char *name, size_t robotID);

    tf2::TransformBroadcaster tf;

    // Last time that we received a velocity command
    ros::Time base_last_cmd;
    ros::Duration base_watchdog_timeout;

    // Current simulation time
    ros::Time sim_time;

  public:
    // Constructor; stage itself needs argc/argv.  fname is the .world file
    // that stage should load.
  StageComponent(const std::string& instance_name, darc::Node::Ptr node);

    ~StageComponent();

    // Subscribe to models of interest.  Currently, we find and subscribe
    // to the first 'laser' model and the first 'position' model.  Returns
    // 0 on success (both models subscribed), -1 otherwise.
    int SubscribeModels();

    // Our callback
    void WorldCallback();

    // Do one update of the world.  May pause if the next update time
    // has not yet arrived.
    bool UpdateWorld();

    // Message callback for a MsgBaseVel message, which set velocities.
    void cmdvelReceived(int idx, const boost::shared_ptr<geometry_msgs::Twist const>& msg);

    // The main simulator object
    Stg::World* world;
};

// since stageros is single-threaded, this is OK. revisit if that changes!
const char *
StageComponent::mapName(const char *name, size_t robotID)
{
  if (positionmodels.size() > 1)
  {
    static char buf[100];
    snprintf(buf, sizeof(buf), "/robot_%u/%s", (unsigned int)robotID, name);
    return buf;
  }
  else
    return name;
}

void
StageComponent::ghfunc(Stg::Model* mod, StageComponent* node)
{
  if (dynamic_cast<Stg::ModelRanger *>(mod))
    node->lasermodels.push_back(dynamic_cast<Stg::ModelRanger *>(mod));
  if (dynamic_cast<Stg::ModelPosition *>(mod))
    node->positionmodels.push_back(dynamic_cast<Stg::ModelPosition *>(mod));
}

void
StageComponent::cmdvelReceived(int idx, const boost::shared_ptr<geometry_msgs::Twist const>& msg)
{
  boost::mutex::scoped_lock lock(msg_lock);
  this->positionmodels[idx]->SetSpeed(msg->linear.x, 
                                      msg->linear.y, 
                                      msg->angular.z);
  this->base_last_cmd = this->sim_time;
}

StageComponent::StageComponent(const std::string& instance_name, darc::Node::Ptr node) :
  darc::Component(instance_name, node),
  tf(this),
  timer_(this, boost::bind(&StageComponent::UpdateWorld, this), boost::posix_time::seconds(1))
{
  this->sim_time.fromSec(0.0);
  this->base_last_cmd.fromSec(0.0);
  double t = 0.2;
  this->base_watchdog_timeout.fromSec(t);

  // We'll check the existence of the world file, because libstage doesn't
  // expose its failure to open it.  Could go further with checks (e.g., is
  // it readable by this user).
  struct stat s;
  //  const char * fname = "/opt/ros/electric/stacks/stage/world/willow-erratic.world";
  const char * fname = "/opt/stage4/share/stage/worlds/simple.world";

  if(stat(fname, &s) != 0)
  {
    std::cout << "The world file %s does not exist.\n";
    exit(2);
  }

  // initialize libstage
  int argc = 0;
  char ** argv = 0;
  Stg::Init(&argc, &argv );

  if(true)
    this->world = new Stg::WorldGui(800, 700, "Stage (ROS)");
  else
    this->world = new Stg::World();

  // Apparently an Update is needed before the Load to avoid crashes on
  // startup on some systems.
  //  this->UpdateWorld();
  this->world->Load(fname);
  std::cout << "X\n";
  // We add our callback here, after the Update, so avoid our callback
  // being invoked before we're ready.
  this->world->AddUpdateCallback((Stg::world_callback_t)s_update, this);
  std::cout << "X\n";

  this->world->ForEachDescendant((Stg::model_callback_t)ghfunc, this);
  if (lasermodels.size() != positionmodels.size())
  {
    std::cout << "number of position models and laser models must be equal in the world file." << lasermodels.size() << " " << positionmodels.size() << std::endl;
    exit(1);
  }
  size_t numRobots = positionmodels.size();
  std::cout << "found " << numRobots<< " position/laser pair%s in the file\n";

  this->laserMsgs = new sensor_msgs::LaserScan[numRobots];
  this->odomMsgs = new nav_msgs::Odometry[numRobots];
  this->groundTruthMsgs = new nav_msgs::Odometry[numRobots];

  this->SubscribeModels();
}


// Subscribe to models of interest.  Currently, we find and subscribe
// to the first 'laser' model and the first 'position' model.  Returns
// 0 on success (both models subscribed), -1 otherwise.
//
// Eventually, we should provide a general way to map stage models onto ROS
// topics, similar to Player .cfg files.
int
StageComponent::SubscribeModels()
{
  for (size_t r = 0; r < this->positionmodels.size(); r++)
  {
    if(this->lasermodels[r])
    {
      this->lasermodels[r]->Subscribe();
    }
    else
    {
      ROS_ERROR("no laser");
      return(-1);
    }
    if(this->positionmodels[r])
    {
      this->positionmodels[r]->Subscribe();
    }
    else
    {
      ROS_ERROR("no position");
      return(-1);
    }
    laser_pubs_.push_back( new darc::pubsub::Publisher<sensor_msgs::LaserScan>(this, mapName(BASE_SCAN,r) ) );
    odom_pubs_.push_back( new darc::pubsub::Publisher<nav_msgs::Odometry>(this, mapName(ODOM,r) ) );
    ground_truth_pubs_.push_back( new darc::pubsub::Publisher<nav_msgs::Odometry>(this, mapName(BASE_POSE_GROUND_TRUTH,r) ) );
    cmdvel_subs_.push_back( new darc::pubsub::Subscriber<geometry_msgs::Twist>(this, mapName(CMD_VEL,r), boost::bind(&StageComponent::cmdvelReceived, this, r, _1)));
  }
  //  clock_pub_ = n_.advertise<rosgraph_msgs::Clock>("/clock",10);
  return(0);
}

StageComponent::~StageComponent()
{
  delete[] laserMsgs;
  delete[] odomMsgs;
  delete[] groundTruthMsgs;
}

bool
StageComponent::UpdateWorld()
{
  std::cout << "UpdateWorld" << std::endl;
  return this->world->Update();
}

void
StageComponent::WorldCallback()
{
  boost::mutex::scoped_lock lock(msg_lock);

  this->sim_time.fromSec(world->SimTimeNow() / 1e6);
  // We're not allowed to publish clock==0, because it used as a special
  // value in parts of ROS, #4027.
  if(this->sim_time.sec == 0 && this->sim_time.nsec == 0)
  {
    ROS_DEBUG("Skipping initial simulation step, to avoid publishing clock==0");
    return;
  }

  // TODO make this only affect one robot if necessary
  if((this->base_watchdog_timeout.toSec() > 0.0) &&
      ((this->sim_time - this->base_last_cmd) >= this->base_watchdog_timeout))
  {
    for (size_t r = 0; r < this->positionmodels.size(); r++)
      this->positionmodels[r]->SetSpeed(0.0, 0.0, 0.0);
  }

  // Get latest laser data
  for (size_t r = 0; r < this->lasermodels.size(); r++)
		{
			const std::vector<Stg::ModelRanger::Sensor>& sensors = this->lasermodels[r]->GetSensors();
		
		if( sensors.size() > 1 )
			ROS_WARN( "ROS Stage currently supports rangers with 1 sensor only." );

		// for now we access only the zeroth sensor of the ranger - good
		// enough for most laser models that have a single beam origin
		const Stg::ModelRanger::Sensor& s = sensors[0];
		
    if( s.ranges.size() )
			{
      // Translate into ROS message format and publish
      this->laserMsgs[r].angle_min = -s.fov/2.0;
      this->laserMsgs[r].angle_max = +s.fov/2.0;
      this->laserMsgs[r].angle_increment = s.fov/(double)(s.sample_count-1);
      this->laserMsgs[r].range_min = s.range.min;
      this->laserMsgs[r].range_max = s.range.max;
      this->laserMsgs[r].ranges.resize(s.ranges.size());
      this->laserMsgs[r].intensities.resize(s.intensities.size());
			
      for(unsigned int i=0; i<s.ranges.size(); i++)
				{
					this->laserMsgs[r].ranges[i] = s.ranges[i];
					this->laserMsgs[r].intensities[i] = (uint8_t)s.intensities[i];
				}
			
      this->laserMsgs[r].header.frame_id = mapName("base_laser_link", r);
      this->laserMsgs[r].header.stamp = sim_time;
      sensor_msgs::LaserScanPtr my_scan(new sensor_msgs::LaserScan(this->laserMsgs[r]));
      this->laser_pubs_[r]->publish(my_scan);
			}

    // Also publish the base->base_laser_link Tx.  This could eventually move
    // into being retrieved from the param server as a static Tx.
    Stg::Pose lp = this->lasermodels[r]->GetPose();
    btQuaternion laserQ;
    laserQ.setRPY(0.0, 0.0, lp.a);
    btVector3 axis = laserQ.getAxis();
    geometry_msgs::TransformStamped tf_laser;
    tf_laser.header.frame_id = "base_link";
    tf_laser.child_frame_id = "base_laser_link";
    tf_laser.transform.translation.x = lp.x;
    tf_laser.transform.translation.y = lp.y;
    tf_laser.transform.translation.z = 0.15;
    tf_laser.transform.rotation.x = axis.x();
    tf_laser.transform.rotation.y = axis.y();
    tf_laser.transform.rotation.z = axis.z();
    tf_laser.transform.rotation.w = laserQ.getW();
    tf.sendTransform(tf_laser);

    /*    tf.sendTransform(geometry_msgs::TransformStamped(txLaser, sim_time,
                                          mapName("base_link", r),
                                          mapName("base_laser_link", r)));
    */
    // Send the identity transform between base_footprint and base_link

    // todo identity transform from footprint to link
    //   btTransform txIdentity(tf::createIdentityQuaternion(),
    //                          tf::Point(0, 0, 0));
    tf.sendTransform(tf_laser);/*tf::StampedTransform(txIdentity,
                                          sim_time,
                                          mapName("base_footprint", r),
                                          mapName("base_link", r)));
			       */
    // Get latest odometry data
    // Translate into ROS message format and publish
    this->odomMsgs[r].pose.pose.position.x = 0;//this->positionmodels[r]->est_pose.x;
    this->odomMsgs[r].pose.pose.position.y = 0;//this->positionmodels[r]->est_pose.y;
    //    this->odomMsgs[r].pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->positionmodels[r]->est_pose.a);
    Stg::Velocity v = this->positionmodels[r]->GetVelocity();
    this->odomMsgs[r].twist.twist.linear.x = v.x;
    this->odomMsgs[r].twist.twist.linear.y = v.y;
    this->odomMsgs[r].twist.twist.angular.z = v.a;

    //@todo Publish stall on a separate topic when one becomes available
    //this->odomMsgs[r].stall = this->positionmodels[r]->Stall();
    //
    this->odomMsgs[r].header.frame_id = mapName("odom", r);
    this->odomMsgs[r].header.stamp = sim_time;

    this->odom_pubs_[r]->publish(boost::shared_ptr<nav_msgs::Odometry>( new nav_msgs::Odometry(this->odomMsgs[r]) ) );
		}
    /* todo
    // broadcast odometry transform
    tf::Quaternion odomQ;
    tf::quaternionMsgToTF(odomMsgs[r].pose.pose.orientation, odomQ);
    tf::Transform txOdom(odomQ, 
                         tf::Point(odomMsgs[r].pose.pose.position.x,
                                   odomMsgs[r].pose.pose.position.y, 0.0));
    tf.sendTransform(tf::StampedTransform(txOdom, sim_time,
                                          mapName("odom", r),
                                          mapName("base_footprint", r)));

    // Also publish the ground truth pose and velocity
    Stg::Pose gpose = this->positionmodels[r]->GetGlobalPose();
    Stg::Velocity gvel = this->positionmodels[r]->GetGlobalVelocity();
    // Note that we correct for Stage's screwed-up coord system.
    tf::Quaternion q_gpose;
    q_gpose.setRPY(0.0, 0.0, gpose.a-M_PI/2.0);
    tf::Transform gt(q_gpose, tf::Point(gpose.y, -gpose.x, 0.0));
    tf::Quaternion q_gvel;
    q_gvel.setRPY(0.0, 0.0, gvel.a-M_PI/2.0);
    tf::Transform gv(q_gvel, tf::Point(gvel.y, -gvel.x, 0.0));

    this->groundTruthMsgs[r].pose.pose.position.x     = gt.getOrigin().x();
    this->groundTruthMsgs[r].pose.pose.position.y     = gt.getOrigin().y();
    this->groundTruthMsgs[r].pose.pose.position.z     = gt.getOrigin().z();
    this->groundTruthMsgs[r].pose.pose.orientation.x  = gt.getRotation().x();
    this->groundTruthMsgs[r].pose.pose.orientation.y  = gt.getRotation().y();
    this->groundTruthMsgs[r].pose.pose.orientation.z  = gt.getRotation().z();
    this->groundTruthMsgs[r].pose.pose.orientation.w  = gt.getRotation().w();
    this->groundTruthMsgs[r].twist.twist.linear.x = gv.getOrigin().x();
    this->groundTruthMsgs[r].twist.twist.linear.y = gv.getOrigin().y();
/    //this->groundTruthMsgs[r].twist.twist.angular.z = tf::getYaw(gv.getRotation());
    //this->groundTruthMsgs[r].twist.twist.linear.x = gvel.x;
    //this->groundTruthMsgs[r].twist.twist.linear.y = gvel.y;
    this->groundTruthMsgs[r].twist.twist.angular.z = gvel.a;

    this->groundTruthMsgs[r].header.frame_id = mapName("odom", r);
    this->groundTruthMsgs[r].header.stamp = sim_time;

    this->ground_truth_pubs_[r].publish(this->groundTruthMsgs[r]);
  }

  this->clockMsg.clock = sim_time;
  this->clock_pub_.publish(this->clockMsg);
    */
}

static bool quit = false;
void
sigint_handler(int num)
{
  quit = true;
}

DARC_REGISTER_COMPONENT(StageComponent)
