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
  darc::pubsub::Publisher<sensor_msgs::LaserScan> laser_pub_;
  darc::pubsub::Publisher<nav_msgs::Odometry> odom_pub_;
  darc::pubsub::Publisher<nav_msgs::Odometry> truth_pub_;
  darc::pubsub::Subscriber<geometry_msgs::Twist> vel_sub_;

  tf2::TransformBroadcaster tf_broadcaster_;

protected:
  void velHandler(geometry_msgs::Twist::ConstPtr vel)
  {
    robot_->SetSpeed(vel->linear.x, vel->linear.y, vel->angular.z);
  }

public:
  StageRobot(darc::Owner * owner, Stg::ModelPosition* robot, Stg::ModelRanger* sensor ) :
    darc::Subcomponent(owner),
    robot_(robot),
    sensor_(sensor),
    laser_pub_(this, "/scan"),
    odom_pub_(this, "/odom"),
    truth_pub_(this, "/truth"),
    vel_sub_(this, "/cmd_vel", boost::bind(&StageRobot::velHandler, this, _1) ),
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
    tf.header.frame_id = "/odom";
    tf.child_frame_id = "/base";
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

    // Estimated Pose Odometry
    nav_msgs::Odometry::Ptr odom( new nav_msgs::Odometry() );
    odom->header = tf.header;
    odom->pose.pose.position.x = robot_->est_pose.x;
    odom->pose.pose.position.y = robot_->est_pose.y;
    odom->pose.pose.position.z = robot_->est_pose.z;
    odom->pose.pose.orientation.x = rotation.x();
    odom->pose.pose.orientation.y = rotation.y();
    odom->pose.pose.orientation.z = rotation.z();
    odom->pose.pose.orientation.w = rotation.w();
    odom_pub_.publish(odom);

    // Ground Truth
    Stg::Pose truth_pose = robot_->GetGlobalPose();
    nav_msgs::Odometry::Ptr truth( new nav_msgs::Odometry() );

    truth->header = tf.header;
    truth->pose.pose.position.x = truth_pose.x;
    truth->pose.pose.position.y = truth_pose.y;
    truth->pose.pose.position.z = truth_pose.z;
    rotation.setRPY(0.0, 0.0, truth_pose.a);
    truth->pose.pose.orientation.x = rotation.x();
    truth->pose.pose.orientation.y = rotation.y();
    truth->pose.pose.orientation.z = rotation.z();
    truth->pose.pose.orientation.w = rotation.w();

    truth_pub_.publish(truth);

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
    scan->header.frame_id = "/base";
    scan->header.stamp = ros::Time::now();
    laser_pub_.publish(scan);

  }
};

}
