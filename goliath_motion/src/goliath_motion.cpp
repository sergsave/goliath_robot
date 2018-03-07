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
    legs_pos_ = body_.getDefaultLegsPos();

    // create public's publisher and subscriber
    legs_pos_sub_ = nh_.subscribe("legs_position", POS_AND_POSE_QUEUE_SZ,
                                  &GoliathMotion::legsPositionCallback, this);

    body_pose_sub_ = nh_.subscribe("body_pose", POS_AND_POSE_QUEUE_SZ,
                                   &GoliathMotion::bodyPoseCallback, this);

    jnt_pub_ =
        nh_.advertise<sensor_msgs::JointState>("joint_states", JNT_QUEUE_SZ);

    jnt_tim_ = nh_.createTimer(ros::Duration(JNT_ST_TIMER_PERIOD),
                               &GoliathMotion::jointStateTimerCallback, this);
    jnt_tim_.stop();

    tf_tim_ = nh_.createTimer(ros::Duration(JNT_ST_TIMER_PERIOD),
                              &GoliathMotion::tfTimerCallback, this);

    tf_tim_.stop();

    jnt_traj_pub_ = private_nh_.advertise<trajectory_msgs::JointTrajectory>(
        "command", JNT_QUEUE_SZ);
  }

private:
  enum QueueSize
  {
    POS_AND_POSE_QUEUE_SZ = 20,
    JNT_QUEUE_SZ = 20
  };

  void tfTimerCallback(const ros::TimerEvent& tim_ev)
  {
    static goliath_msgs::BodyPose curr_pose;
    static goliath_msgs::BodyPose delta_pose;
    static std::size_t itter_numb;

    if (tf_itteration_cnt_ == 0)
    {
      itter_numb = MOVE_TIME_STEP / JNT_ST_TIMER_PERIOD;
      delta_pose.position.x =
          (goal_pose_.position.x - curr_pose.position.x) / itter_numb;
      delta_pose.position.y =
          (goal_pose_.position.y - curr_pose.position.y) / itter_numb;
      delta_pose.position.z =
          (goal_pose_.position.z - curr_pose.position.z) / itter_numb;

      delta_pose.roll = (goal_pose_.roll - curr_pose.roll) / itter_numb;
      delta_pose.pitch = (goal_pose_.pitch - curr_pose.pitch) / itter_numb;
      delta_pose.yaw = (goal_pose_.yaw - curr_pose.yaw) / itter_numb;
    }
    else if (tf_itteration_cnt_ < itter_numb)
    {
      curr_pose.position.x += delta_pose.position.x;
      curr_pose.position.y += delta_pose.position.y;
      curr_pose.position.z += delta_pose.position.z;

      curr_pose.roll += delta_pose.roll;
      curr_pose.pitch += delta_pose.pitch;
      curr_pose.yaw += delta_pose.yaw;

      publishTransformToGroundFrame(curr_pose);
    }

    if (tf_itteration_cnt_ == itter_numb)
    {
      publishTransformToGroundFrame(curr_pose = goal_pose_);
      tf_tim_.stop();
    }
    else
      tf_itteration_cnt_++;
  }

  void publishTfPose(const goliath_msgs::BodyPose& pose)
  {
    goal_pose_ = pose;
    tf_itteration_cnt_ = 0;
    tf_tim_.start();
  }

  void publishJointState(const trajectory_msgs::JointTrajectory& traj)
  {
    point_cnt_ = itteration_cnt_ = 0;
    goal_traj_ = traj;
    jnt_tim_.start();
  }

  void jointStateTimerCallback(const ros::TimerEvent& tim_ev)
  {
    sensor_msgs::JointState jnt_st;
    static std::vector<double> prev_st;
    static std::vector<double> delta_pos;
    static std::size_t itterations_numb = 0;

    jnt_st.name = goal_traj_.joint_names;

    if (point_cnt_ < goal_traj_.points.size())
    {
      if (itteration_cnt_ == 0)
      {
        jnt_st.position.clear();
        delta_pos.clear();

        itterations_numb =
            goal_traj_.points[point_cnt_].time_from_start.toSec() /
            JNT_ST_TIMER_PERIOD;

        for (std::size_t j = 0;
             j != goal_traj_.points[point_cnt_].positions.size(); j++)
        {
          delta_pos.push_back((goal_traj_.points[point_cnt_].positions[j] -
                               (prev_st.empty() ? 0 : prev_st[j])) /
                              itterations_numb);
        }
      }
      else if (itteration_cnt_ < itterations_numb)
      {
        jnt_st.header.stamp = ros::Time::now();

        for (std::size_t j = 0;
             j != goal_traj_.points[point_cnt_].positions.size(); j++)
          jnt_st.position.push_back((prev_st.empty() ? 0 : prev_st[j]) +
                                    delta_pos[j]);

        jnt_pub_.publish(jnt_st);

        prev_st = jnt_st.position;
        jnt_st.position.clear();
      }

      if (itteration_cnt_ == itterations_numb)
      {
        jnt_st.header.stamp = ros::Time::now();
        prev_st = jnt_st.position = goal_traj_.points[point_cnt_].positions;

        jnt_pub_.publish(jnt_st);

        itteration_cnt_ = 0;
        point_cnt_++;
      }
      else
        itteration_cnt_++;
    }
    else
      jnt_tim_.stop();
  }

  void bodyPoseCallback(const goliath_msgs::BodyPose& pose)
  {
    sensor_msgs::JointState jnt_st;

    jnt_st.header.stamp = ros::Time::now();

    try
    {
      body_.calculateJntAngles(pose, legs_pos_, jnt_st);
    }
    catch (std::logic_error e)
    {
      ROS_ERROR_STREAM(e.what());
      return;
    }

    trajectory_msgs::JointTrajectory jnt_traj;
    trajectory_msgs::JointTrajectoryPoint point;

    jnt_traj.header.stamp = ros::Time::now();
    jnt_traj.joint_names = jnt_st.name;

    point.positions = jnt_st.position;
    point.time_from_start = ros::Duration(MOVE_TIME_STEP);

    jnt_traj.points.push_back(point);
    jnt_traj_pub_.publish(jnt_traj);

    publishTfPose(pose);
    publishJointState(jnt_traj);

  }

  void legsPositionCallback(const goliath_msgs::LegsPosition& pos)
  {
    sensor_msgs::JointState jnt_st;

    jnt_st.header.stamp = ros::Time::now();

    try
    {
      body_.calculateJntAngles(pos, jnt_st);
    }
    catch (std::logic_error e)
    {
      ROS_ERROR_STREAM(e.what());
      return;
    }

    trajectory_msgs::JointTrajectory jnt_traj;
    trajectory_msgs::JointTrajectoryPoint point;

    jnt_traj.header.stamp = ros::Time::now();
    jnt_traj.joint_names = jnt_st.name;

    point.positions = jnt_st.position;
    point.time_from_start = ros::Duration(MOVE_TIME_STEP);

    jnt_traj.points.push_back(point);

    jnt_traj_pub_.publish(jnt_traj);
    publishJointState(jnt_traj);

    goliath_msgs::BodyPose default_pose;
    publishTfPose(default_pose);

    legs_pos_ = pos;
  }

  void publishTransformToGroundFrame()
  {
    tf::Transform transform;
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    transform.setOrigin(tf::Vector3(0, 0, body_.getClearance()));
    tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                              "body_link", "ground"));
  }

  void publishTransformToGroundFrame(const goliath_msgs::BodyPose& pose)
  {
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion q;
    q.setRPY(-pose.roll, -pose.pitch, -pose.yaw);
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

  static const double JNT_ST_TIMER_PERIOD;
  static const double TF_TIMER_PERIOD;
  static const double MOVE_TIME_STEP;
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher jnt_pub_;
  ros::Subscriber legs_pos_sub_;
  ros::Subscriber body_pose_sub_;
  BodyKinematics body_;

  goliath_msgs::LegsPosition legs_pos_;

  ros::Publisher jnt_traj_pub_;

  // use for visualisation
  tf::TransformBroadcaster tf_br_;

  ros::Timer jnt_tim_;
  trajectory_msgs::JointTrajectory goal_traj_;
  std::size_t point_cnt_, itteration_cnt_;

  ros::Timer tf_tim_;
  goliath_msgs::BodyPose goal_pose_;
  std::size_t tf_itteration_cnt_;
};

const double GoliathMotion::JNT_ST_TIMER_PERIOD = 0.05;
const double GoliathMotion::TF_TIMER_PERIOD = 0.05;
const double GoliathMotion::MOVE_TIME_STEP = 0.5;

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
