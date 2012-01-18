/*
 * Copyright (c) 2011, Prevas A/S
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Prevas A/S nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * DARC Component class
 *
 * \author Morten Kjaergaard
 */

#include <darc/subcomponent.h>
#include <darc/pubsub/publisher.h>
#include <darc/pubsub/subscriber.h>

#include <tf2_darc/transform_broadcaster.h>

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <LinearMath/btQuaternion.h>
#include <LinearMath/btTransform.h>

namespace stage_darc
{

class StageRobot : public darc::Subcomponent
{
protected:
  Stg::ModelPosition* robot_;
  Stg::ModelRanger* sensor_;
  darc::pubsub::Publisher<sensor_msgs::LaserScan> laser_pub_;/*
   darc::pubsub::Publisher<nav_msgs::Odometry> odom_pub_;
  darc::pubsub::Publisher<nav_msgs::Odometry> ground_truth_pub_;
  darc::pubsub::Subscriber<geometry_msgs::Twist> cmdvel_sub_;
  */
  tf2::TransformBroadcaster tf_broadcaster_;

public:
  StageRobot(darc::Owner * owner, Stg::ModelPosition* robot, Stg::ModelRanger* sensor ) :
    darc::Subcomponent(owner),
    robot_(robot),
    sensor_(sensor),
    laser_pub_(this, "/scan"),
    tf_broadcaster_(this)
  {
    robot_->Subscribe();
    sensor->Subscribe();
    if( sensor_->GetSensors().size() != 1)
    {
      std::cout << "Only support 1 sensor" << std::endl;
      exit(1);
    }
  }

  ~StageRobot() {};

  void update()
  {
    // Estimated Pose (TF)
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base";
    tf.transform.translation.x = robot_->est_pose.x;
    tf.transform.translation.y = robot_->est_pose.y;
    tf.transform.translation.z = robot_->est_pose.z;
    btQuaternion rotation;
    rotation.setRPY(0.0, 0.0, robot_->est_pose.a);
    tf.transform.rotation.x = rotation.x();
    tf.transform.rotation.y = rotation.y();
    tf.transform.rotation.z = rotation.z();
    tf.transform.rotation.w = rotation.w();
    tf_broadcaster_.sendTransform(tf);

    // Laserscan
    sensor_msgs::LaserScan::Ptr scan( new sensor_msgs::LaserScan() );

    const Stg::ModelRanger::Sensor& s = sensor_->GetSensors().at(0);

    scan->angle_min = -s.fov/2.0;
    scan->angle_max = +s.fov/2.0;
    scan->angle_increment = s.fov/(double)(s.sample_count-1);
    scan->range_min = s.range.min;
    scan->range_max = s.range.max;
    scan->ranges.resize(s.ranges.size());
    scan->intensities.resize(s.intensities.size());
    for(unsigned int i=0; i<s.ranges.size(); i++)
    {
      scan->ranges[i] = s.ranges[i];
      scan->intensities[i] = (uint8_t)s.intensities[i];
    }
    scan->header.frame_id = "base";
    scan->header.stamp = ros::Time::now();
    laser_pub_.publish(scan);

  }
};

#if 0

void
StageComponent::WorldCallback()
{
  std::cout << "WorldCallback" << std::endl;
  return;

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
#endif

}
