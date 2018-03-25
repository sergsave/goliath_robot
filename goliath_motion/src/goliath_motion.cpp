// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

#include "ros/ros.h"
#include <string>
#include <array>
#include <tf/transform_broadcaster.h>
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/Twist.h"
#include "goliath_msgs/LegsVel.h"
#include "body_kinematics.h"
#include "gait_generator.h"

using std::string;
using std::endl;

// this class contain motion control mechanism
// for Goliath - 6 leg's insect's like robot.
class GoliathMotion
{
public:
  GoliathMotion(ros::NodeHandle node)
      : nh_(node), private_nh_("~"),
        last_gait_or_legs_command_time_(ros::Time::now()),
        last_body_command_time_(ros::Time::now())
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
    curr_legs_pos_ = body_.getDefaultLegsPos();
    gait_ = GaitGenerator(body_);

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
      BodyKinematics::LegsPosition new_legs_pos = curr_legs_pos_;
      BodyKinematics::BodyPose new_body_pose = curr_body_pose_;
      urdf::Vector3 new_odom_dist = curr_odom_dist_;

      shiftLegsPos(new_legs_pos, MOVE_TIME_STEP);
      shiftBodyPose(new_body_pose, MOVE_TIME_STEP);
      shiftOdomDist(new_odom_dist, MOVE_TIME_STEP);

      // geometry_msgs::Twist test;
      // test.linear.x = 0.02;
      gait_.accretion(gait_velocity_, new_legs_pos, MOVE_TIME_STEP);

      trajectory_msgs::JointTrajectory traj;

      if (createJntTraj(new_legs_pos, new_body_pose, traj))
      {
        jnt_traj_pub_.publish(traj);
        waitAndPublishTf(MOVE_TIME_STEP, curr_body_pose_,
                         curr_odom_dist_);

        curr_legs_pos_ = new_legs_pos;
        curr_body_pose_ = new_body_pose;
        curr_odom_dist_ = new_odom_dist;
      }
      else
        ros::Duration(MOVE_TIME_STEP).sleep();

      if (ros::Time::now() - last_gait_or_legs_command_time_ >
          ros::Duration(MOVE_TIME))
      {
        // set vel to zero
        legs_velocity_ = goliath_msgs::LegsVel();
        gait_velocity_ = geometry_msgs::Twist();
      }

      if (ros::Time::now() - last_body_command_time_ > ros::Duration(MOVE_TIME))
      {
        // set vel to zero
        body_velocity_ = geometry_msgs::Twist();
      }

      ros::spinOnce();
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

  void shiftLegsPos(BodyKinematics::LegsPosition& new_lp, double time)
  {
    for (std::size_t l = 0; l != new_lp.size(); ++l)
    {
      LegKinematics::LegPos iter(time * legs_velocity_.velocities[l].x,
                                 time * legs_velocity_.velocities[l].y,
                                 time * legs_velocity_.velocities[l].z);
      new_lp[l] = new_lp[l] + iter;
    }
  }

  void shiftBodyPose(BodyKinematics::BodyPose& new_bp, double time)
  {
    double roll, pitch, yaw;
    new_bp.rotation.getRPY(roll, pitch, yaw);
    new_bp.rotation.setFromRPY(roll + time * body_velocity_.angular.x,
                               pitch + time * body_velocity_.angular.y,
                               yaw + time * body_velocity_.angular.z);

    urdf::Vector3 iter(time * body_velocity_.linear.x,
                       time * body_velocity_.linear.y,
                       time * body_velocity_.linear.z);
    new_bp.position = new_bp.position + iter;
  }

  void shiftOdomDist(urdf::Vector3& new_dist, double time)
  {
    urdf::Vector3 iter(time * gait_velocity_.linear.x,
                       time * gait_velocity_.linear.y,
                       time * gait_velocity_.linear.z);

    new_dist = new_dist + iter;
  }

  bool createJntTraj(const BodyKinematics::LegsPosition& lp,
                     const BodyKinematics::BodyPose& bp,
                     trajectory_msgs::JointTrajectory& traj)
  {
    trajectory_msgs::JointTrajectoryPoint traj_point;

    try
    {
      body_.calculateJntAngles(bp, lp, traj_point);
    }
    catch (std::logic_error e)
    {
      ROS_ERROR_STREAM(e.what());
      return false;
    }

    body_.getLegsJntName(traj.joint_names);
    traj.header.stamp = ros::Time::now();

    traj_point.time_from_start = ros::Duration(MOVE_TIME_STEP);
    traj.points.push_back(traj_point);

    return true;
  }

  void waitAndPublishTf(double time, BodyKinematics::BodyPose pose,
                        urdf::Vector3 od_dist)
  {
    const double tf_pub_period = time / TF_NUMBERS_PER_STEP;

    for (std::size_t i = 0; i != TF_NUMBERS_PER_STEP; ++i)
    {
      publishTransformToBase(pose);
      publishTransformToOdom(od_dist);
      shiftBodyPose(pose, tf_pub_period);
      shiftOdomDist(od_dist, tf_pub_period);
      ros::Duration(tf_pub_period).sleep();
    }
  }

  void publishTransformToBase(const BodyKinematics::BodyPose& pose)
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
                                              "center_of_rotation", "base"));
  }

  void publishTransformToOdom(const urdf::Vector3& dist)
  {
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(-dist.x, -dist.y, 0));

    tf_br_.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "base", "odom"));
  }

  static const int VEL_QUEUE_SZ = 10;
  static const int JNT_TRAJ_QUEUE_SZ = 10;
  static const int TF_NUMBERS_PER_STEP = 5;
  static const double MOVE_TIME_STEP, MOVE_TIME;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  BodyKinematics body_;
  GaitGenerator gait_;

  ros::Subscriber gait_vel_sub_;
  ros::Subscriber body_vel_sub_;
  ros::Subscriber legs_vel_sub_;
  ros::Publisher jnt_traj_pub_;

  geometry_msgs::Twist gait_velocity_;
  geometry_msgs::Twist body_velocity_;
  goliath_msgs::LegsVel legs_velocity_;

  ros::Time last_gait_or_legs_command_time_;
  ros::Time last_body_command_time_;

  BodyKinematics::LegsPosition curr_legs_pos_;
  BodyKinematics::BodyPose curr_body_pose_;
  urdf::Vector3 curr_odom_dist_;

  // use for visualisation
  tf::TransformBroadcaster tf_br_;
};

const double GoliathMotion::MOVE_TIME_STEP = 0.1;
const double GoliathMotion::MOVE_TIME = 0.5;

int main(int argc, char** argv)
{
  // uncomment for debug purpose
  // sleep(2);

  // Let's start ROS!
  ros::init(argc, argv, "goliath_motion");
  ros::NodeHandle node;

  GoliathMotion goliath_motion(node);

  goliath_motion.spin();

  return 0;
}
