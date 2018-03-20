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

      if (ros::Time::now() - last_update_time_ >
          ros::Duration(MOVE_TIME_STEP))
      {
        BodyKinematics::LegsPosition new_legs_pos = curr_legs_pos_;
        BodyKinematics::BodyPose new_body_pose = curr_body_pose_;
        trajectory_msgs::JointTrajectory traj;

        shiftLegsPos(new_legs_pos);
        shiftBodyPose(new_body_pose);

        if (createJntTraj(new_legs_pos, new_body_pose, traj))
        {
          publishTransformToGroundFrame(curr_body_pose_);
          jnt_traj_pub_.publish(traj);
          curr_body_pose_ = new_body_pose;
          curr_legs_pos_ = new_legs_pos;
        }

        last_update_time_ = ros::Time::now();
      }

      if (ros::Time::now() - last_gait_or_legs_command_time_ >
          ros::Duration(MOVE_TIME))
      {
        // set vel to zero
        legs_velocity_ = goliath_msgs::LegsVel();
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

  void shiftLegsPos(BodyKinematics::LegsPosition& new_lp)
  {
    for (std::size_t l = 0; l != new_lp.size(); ++l)
    {
      LegKinematics::LegPos iter(
          MOVE_TIME_STEP * legs_velocity_.velocities[l].x,
          MOVE_TIME_STEP * legs_velocity_.velocities[l].y,
          MOVE_TIME_STEP * legs_velocity_.velocities[l].z);
      new_lp[l] = new_lp[l] + iter;
    }
  }

  void shiftBodyPose(BodyKinematics::BodyPose& new_bp)
  {
    double roll, pitch, yaw;
    new_bp.rotation.getRPY(roll, pitch, yaw);
    new_bp.rotation.setFromRPY(roll + MOVE_TIME_STEP * body_velocity_.angular.x,
                               pitch +
                                   MOVE_TIME_STEP * body_velocity_.angular.y,
                               yaw + MOVE_TIME_STEP * body_velocity_.angular.z);

    urdf::Vector3 iter(MOVE_TIME_STEP * body_velocity_.linear.x,
                       MOVE_TIME_STEP * body_velocity_.linear.y,
                       MOVE_TIME_STEP * body_velocity_.linear.z);
    new_bp.position = new_bp.position + iter;
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

  void publishTransformToGroundFrame(const BodyKinematics::BodyPose& pose)
  {
#warning "add smooth transform!"
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
  static const double MOVE_TIME_STEP, MOVE_TIME;

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
  ros::Time last_update_time_;

  BodyKinematics::LegsPosition curr_legs_pos_;
  BodyKinematics::BodyPose curr_body_pose_;

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
