// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

#include "ros/ros.h"
#include <string>
#include <array>
#include "hexapod.h"
#include "sensor_msgs/JointState.h"
#include "goliath_msgs/LegPosition.h"
#include "goliath_msgs/BodyPose.h"
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

using std::string;
using std::cout;
using std::endl;

// this class contain motion control mechanism
// for Goliath - 6 leg's insect's like robot.
class GoliathLocomotion
{
public:
  GoliathLocomotion(ros::NodeHandle node) : n_(node)
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

    hexapod_ = Hexapod(model);

    // create public's publisher and subscriber
    pos_sub_ = n_.subscribe("leg_position", POS_QUEUE_SZ,
                            &GoliathLocomotion::legPositionCallback, this);

    body_pos_sub_ = n_.subscribe("body_pose", POS_QUEUE_SZ,
                                 &GoliathLocomotion::bodyPoseCallback, this);
    jnt_pub_ =
        n_.advertise<sensor_msgs::JointState>("joint_states", JNT_QUEUE_SZ);

    marker_pub_ =
        n_.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    points_.header.frame_id = "body_link";
    points_.header.stamp = ros::Time::now();

    points_.ns = "points_and_lines";
    points_.action = visualization_msgs::Marker::ADD;

    points_.id = 0;
    points_.type = visualization_msgs::Marker::POINTS;

    points_.scale.x = points_.scale.y = points_.scale.z = 0.01;
  }

private:
  enum QueueSize
  {
    POS_QUEUE_SZ = 5,
    JNT_QUEUE_SZ = 5
  };

  void bodyPoseCallback(const goliath_msgs::BodyPose& pose)
  {
    Hexapod::JntNames jnt_names = hexapod_.getJntNames();
    Hexapod::Angles angs = {0};

    sensor_msgs::JointState jnt;
    jnt.header.stamp = ros::Time::now();

    RoboLeg::Position debug;
    points_.points.clear();
    // calculate inverse kinematics for each leg
    for (std::size_t n = 0; n != angs.size(); ++n)
    {
      try
      {
        hexapod_.getAnglesFromBodyPose(static_cast<Hexapod::LegType>(n), pose,
                                       angs, debug);
        geometry_msgs::Point p;
        p.x = debug.x;
        p.y = debug.y;
        p.z = debug.z;
        points_.points.push_back(p);
      }
      catch (std::logic_error e)
      {
        ROS_ERROR_STREAM(e.what());
        return;
      }
    }

    // publish msgs to jnt-states topic
    for (std::size_t i = 0; i != angs.size(); ++i)
      for (std::size_t j = 0; j != angs[i].size(); ++j)
      {
        jnt.name.push_back(jnt_names[i][j]);
        jnt.position.push_back(angs[i][j]);
      }
    jnt_pub_.publish(jnt);

    RoboLeg::Position ground = hexapod_.getGround();
    transform_.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion q;
    q.setRPY(-pose.roll, -pose.pitch, -pose.yaw);
    transform_.setRotation(q);

    br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(),
                                           "body_link", "center_of_rotation"));

    q.setRPY(0, 0, 0);
    transform_.setRotation(q);
    transform_.setOrigin(tf::Vector3(ground.x - pose.position.x,
                                     ground.y - pose.position.y,
                                     ground.z - pose.position.z));
    br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(),
                                           "center_of_rotation", "ground"));

    // print all published data for debug - info
    for (std::size_t i = 0; i != jnt.name.size(); ++i)
      ROS_INFO_STREAM(jnt.name[i] << ' ' << jnt.position[i] << std::endl);

    std_msgs::ColorRGBA c;
    c.r = 1.0;
    c.a = 1.0;
    geometry_msgs::Point p;

    points_.color = c;
    marker_pub_.publish(points_);
  }

  void legPositionCallback(const goliath_msgs::LegPosition& pos)
  {
    Hexapod::JntNames jnt_names = hexapod_.getJntNames();
    //WARNING!!! static only for debug
    static Hexapod::Angles angs;

    sensor_msgs::JointState jnt;
    jnt.header.stamp = ros::Time::now();

    RoboLeg::Position leg_pos(pos.position_of_leg.x, pos.position_of_leg.y,
                              pos.position_of_leg.z);
    try
    {
      hexapod_.getAnglesForSingleLeg(
          static_cast<Hexapod::LegType>(pos.number_of_leg), leg_pos, angs);
    }
    catch (std::logic_error e)
    {
      ROS_ERROR_STREAM(e.what());
      return;
    }

    RoboLeg::Position ground = hexapod_.getGround();

    transform_.setOrigin(tf::Vector3(ground.x,
                                     ground.y,
                                     ground.z));
    br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(),
                                           "body_link", "ground"));

    // publish msgs to jnt-states topic
    for (std::size_t i = 0; i != angs.size(); ++i)
      for (std::size_t j = 0; j != angs[i].size(); ++j)
      {
        jnt.name.push_back(jnt_names[i][j]);
        jnt.position.push_back(angs[i][j]);
      }
    jnt_pub_.publish(jnt);

    // print all published data for debug - info
    for (std::size_t i = 0; i != jnt.name.size(); ++i)
      ROS_INFO_STREAM(jnt.name[i] << ' ' << jnt.position[i] << std::endl);
  }

  ros::NodeHandle n_;
  ros::Publisher jnt_pub_;
  ros::Subscriber pos_sub_;
  ros::Subscriber body_pos_sub_;
  Hexapod hexapod_;

  ros::Publisher marker_pub_;
  visualization_msgs::Marker points_;

  // test frame
  tf::TransformBroadcaster br_;
  tf::Transform transform_;
};

int main(int argc, char** argv)
{
  // for debug purpose
  sleep(2);
  // Let's start ROS!
  ros::init(argc, argv, "goliath_kinematics");

  ros::NodeHandle node;
  GoliathLocomotion goliath_locomotion(node);

  // wait
  ros::spin();
  return 0;
}
