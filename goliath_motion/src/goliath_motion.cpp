// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

#include "ros/ros.h"
#include <string>
#include <array>
#include <tf/transform_broadcaster.h>
#include "body_kinematics.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/Twist.h"
#include "goliath_msgs/LegsVel.h"

using std::string;
using std::endl;

// this class contain motion control mechanism
// for Goliath - 6 leg's insect's like robot.
class GoliathMotion
{
public:
  GoliathMotion(ros::NodeHandle node) : nh_(node), private_nh_("~")
  {
    urdf::Model model;

    // use parameter from ROS parameter server for create URDF-model.
    // if an xml robot's description is not added to server, and error
    // will occur
    if (!model.initParam("robot_description"))
    {
      ROS_FATAL_STREAM("Need a \"robot description\" parameter on server"
                       << endl);
      exit(-1);
    }

    body_ = BodyKinematics(model);

    // create public's publisher and subscriber

    gait_vel_sub_ = nh_.subscribe("gait_cmd_vel", VEL_QUEUE_SZ,
                                  &GoliathMotion::gaitVelCallback, this);

    body_vel_sub_ = nh_.subscribe("body_cmd_vel", VEL_QUEUE_SZ,
                                  &GoliathMotion::bodyVelCallback, this);

    legs_vel_sub_ = nh_.subscribe("legs_cmd_vel", VEL_QUEUE_SZ,
                                  &GoliathMotion::legsVelCallback, this);

    jnt_traj_pub_ = private_nh_.advertise<trajectory_msgs::JointTrajectory>(
        "command", JNT_TRAJ_QUEUE_SZ);
  }

  void spin(void)
  {

    while (ros::ok())
    {

      if (ros::Time::now() - last_jnt_traj_pub_time_ >
          ros::Duration(MOVE_TIME_STEP))
      {
        publishJntTraj();
        last_jnt_traj_pub_time_ = ros::Time::now();
      }
    }
  }

private:
  void gaitVelCallback(const geometry_msgs::Twist& tw)
  {
    gait_velocity_ = tw;
    last_gait_or_legs_command_time_ = ros::Time::now();
  }

  void bodyVelCallback(const geometry_msgs::Twist& tw)
  {
    body_velocity_ = tw;
    last_body_command_time_ = ros::Time::now();
  }

  void legsVelCallback(const goliath_msgs::LegsVel& legs_vel)
  {
    legs_velocity_ = legs_vel;
    last_gait_or_legs_command_time_ = ros::Time::now();
  }

  void publishJntTraj(void)
  {
    static BodyKinematics::LegsPosition curr_legs_pos;
    static BodyKinematics::BodyPose curr_body_pose;

    BodyKinematics::LegsPosition new_legs_pos = curr_legs_pos;
    BodyKinematics::BodyPose new_body_pose = curr_body_pose;

    //****************************************************

    trajectory_msgs::JointTrajectoryPoint traj_point;
    trajectory_msgs::JointTrajectory traj;

    body_.getLegsJntName(traj.joint_names);
    traj.header.stamp = ros::Time::now();

    try
    {
      body_.calculateJntAngles(curr_body_pose, curr_legs_pos, traj_point);
    }
    catch (std::logic_error e)
    {
      ROS_ERROR_STREAM(e.what());
      return;
    }

    traj_point.time_from_start = ros::Duration(MOVE_TIME_STEP);
    traj.points.push_back(traj_point);

    jnt_traj_pub_.publish(traj_point);
  }

  void publishTransformToGroundFrame(const BodyKinematics::BodyPose& pose)
  {
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0, 0, 0));

    double roll, pitch, yaw;
    pose.rotation.getRPY(roll, pitch, yaw);

    tf::Quaternion q;
    q.setRPY(-roll, -pitch, -yaw);
    transform.setRotation(q);

    tf_br_.sendTransform(tf::StampedTransform(
        transform, ros::Time::now(), "body_link", "center_of_rotation"));
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    transform.setOrigin(tf::Vector3(-pose.position.x, -pose.position.y,
                                    body_.getClearance() - pose.position.z));
    tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                              "center_of_rotation", "ground"));
  }

  static const int VEL_QUEUE_SZ = 10;
  static const int JNT_TRAJ_QUEUE_SZ = 10;
  static const double MOVE_TIME_STEP;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  BodyKinematics body_;

  ros::Subscriber gait_vel_sub_;
  ros::Subscriber body_vel_sub_;
  ros::Subscriber legs_vel_sub_;
  ros::Publisher jnt_traj_pub_;

  geometry_msgs::Twist gait_velocity_;
  geometry_msgs::Twist body_velocity_;
  goliath_msgs::LegsVel legs_velocity_;

  ros::Time last_gait_or_legs_command_time_;
  ros::Time last_body_command_time_;
  ros::Time last_jnt_traj_pub_time_;

  // use for visualisation
  tf::TransformBroadcaster tf_br_;
};

const double GoliathMotion::MOVE_TIME_STEP = 0.1;

int main(int argc, char** argv)
{
  // uncomment for debug purpose
  // sleep(2);

  // Let's start ROS!
  ros::init(argc, argv, "goliath_motion");
  ros::NodeHandle node;

  GoliathMotion goliath_motion(node);

  ros::spin();

  return 0;
}
