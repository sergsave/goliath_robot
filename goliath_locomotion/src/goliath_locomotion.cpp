// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

#include "urdf/model.h"
#include "ros/ros.h"
#include <string>
#include <iostream>
#include "hexa_leg.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/JointState.h"

using std::string;
using std::cout;
using std::endl;

HexaLeg* p_leg;
ros::Publisher jnt_pub;

void positionCallback(const geometry_msgs::Point32& pos)
{
  HexaLeg::Position hex_pos(pos.x, pos.y, pos.z);
  HexaLeg::Angles hex_angs;
  sensor_msgs::JointState jnt;
  HexaLeg::JntNames names;

  if (p_leg->getAnglesIK(hex_pos, hex_angs) == HexaLeg::OK)
  {
    ROS_INFO_STREAM("Calc IK: cft - [" << hex_angs.coxa_ << ", "
                                       << hex_angs.femur_ << ", "
                                       << hex_angs.tibia_ << "]");
    names = p_leg->getJntNames();
    jnt.header.stamp = ros::Time::now();
    jnt.name.resize(3); //why 3???
    jnt.position.resize(3); //why 3??
    jnt.name[0] = names.coxa_;
    jnt.position[0] = hex_angs.coxa_;
    jnt.name[1] = names.femur_;
    jnt.position[1] = hex_angs.femur_;
    jnt.name[2] = names.tibia_;
    jnt.position[2] = hex_angs.tibia_;
    jnt_pub.publish(jnt);
  }
  else
    ROS_ERROR_STREAM("Calc IK failed!");
}

int main(int argc, char** argv)
{
  // sleep(3); // for debug purpose
  urdf::Model my_model;

  ros::init(argc, argv, "goliath_kinematics");
  ros::NodeHandle n;

  if (argc != 3)
  {
    ROS_ERROR("Need a urdf file as argument");
    ROS_ERROR("Need a leg prefix as argument");
    return -1;
  }
  std::string urdf_file = argv[1];

  if (!my_model.initFile(urdf_file))
  {
    ROS_ERROR("Failed to parse urdf file");
    return -1;
  }
  static HexaLeg leg(argv[2], my_model);
  p_leg = &leg;

  ros::Subscriber position_sub = n.subscribe("position", 100, positionCallback);
  jnt_pub = n.advertise<sensor_msgs::JointState>("joint_states", 5);

  ros::spin();
  return 0;
}
