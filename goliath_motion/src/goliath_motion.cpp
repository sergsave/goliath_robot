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

    jnt_traj_sub_ =
        private_nh_.subscribe("command", POS_AND_POSE_QUEUE_SZ,
                              &GoliathMotion::trajectoryCallback, this);

    jnt_traj_pub_ = private_nh_.advertise<trajectory_msgs::JointTrajectory>(
        "command", JNT_QUEUE_SZ);
  }

  void spin() { ros::spin(); }

private:
  enum QueueSize
  {
    POS_AND_POSE_QUEUE_SZ = 20,
    JNT_QUEUE_SZ = 20
  };

  void trajectoryCallback(const trajectory_msgs::JointTrajectory& traj)
  {
    const double RATE_VAL = 20;
    sensor_msgs::JointState jnt_st;
    static std::vector<double> prev_st;

    jnt_st.name = traj.joint_names;

    ros::Rate loop_rate(RATE_VAL);

    for (auto p : traj.points)
    {
      std::size_t itterations_numb = p.time_from_start.toSec() * RATE_VAL;
      std::vector<double> delta_pos;

      for (std::size_t j = 0; j != p.positions.size(); j++)
      {
        delta_pos.push_back(
            (p.positions[j] - (prev_st.empty() ? 0 : prev_st[j])) /
            itterations_numb);
      }

      for (std::size_t i = 1; i <= itterations_numb; i++)
      {
        loop_rate.sleep();
        jnt_st.header.stamp = ros::Time::now();

        for (std::size_t j = 0; j != p.positions.size(); j++)
          jnt_st.position.push_back((prev_st.empty() ? 0 : prev_st[j]) +
                                    i * delta_pos[j]);

        publishTransformToGroundFrame();
        jnt_pub_.publish(jnt_st);
        jnt_st.position.clear();
      }

      jnt_st.header.stamp = ros::Time::now();
      prev_st = jnt_st.position = p.positions;

      jnt_pub_.publish(jnt_st);
      jnt_st.position.clear();
    }
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

    publishTransformToGroundFrame(pose);

    trajectory_msgs::JointTrajectory jnt_traj;
    trajectory_msgs::JointTrajectoryPoint point;

    jnt_traj.header.stamp = ros::Time::now();
    jnt_traj.joint_names = jnt_st.name;

    point.positions = jnt_st.position;
    point.time_from_start = ros::Duration(0.0);

    jnt_traj.points.push_back(point);
    jnt_traj_pub_.publish(jnt_traj);
  }

  void legsPositionCallback(const goliath_msgs::LegsPosition& pos)
  {
    legs_pos_ = pos;
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
    point.time_from_start = ros::Duration(0.75);

    jnt_traj.points.push_back(point);
    jnt_traj_pub_.publish(jnt_traj);
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

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher jnt_pub_;
  ros::Subscriber legs_pos_sub_;
  ros::Subscriber body_pose_sub_;
  BodyKinematics body_;

  goliath_msgs::LegsPosition legs_pos_;

  ros::Publisher jnt_traj_pub_;
  ros::Subscriber jnt_traj_sub_;

  // use for visualisation
  tf::TransformBroadcaster tf_br_;
};

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
