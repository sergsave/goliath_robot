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
#include "goliath_msgs/Pose.h"

using std::string;
using std::cout;
using std::endl;

class GoliathLocomotion
{
public:
  GoliathLocomotion(string robot_urdf, ros::NodeHandle node) : n_(node)
  {
    urdf::Model model;
    if (!model.initFile(robot_urdf))
    {
      ROS_ERROR_STREAM("Failed, \"" << robot_urdf << "\" is not a URDF file."
                                    << endl);
      exit(-1);
    }
    hexapod_ = Hexapod(model);

    pos_sub_ = n_.subscribe("leg_positions", POS_QUEUE_SZ,
                            &GoliathLocomotion::legPositionCallback, this);
    jnt_pub_ =
        n_.advertise<sensor_msgs::JointState>("joint_states", JNT_QUEUE_SZ);
  }

  void test()
  {
    goliath_msgs::Pose p;

    legPositionCallback(p);
  }

private:
  enum QueueSize
  {
    POS_QUEUE_SZ = 5,
    JNT_QUEUE_SZ = 5
  };

  void legPositionCallback(const goliath_msgs::Pose& pos)
  {
    std::vector<RoboLeg::Position> legs_pos;

    for (auto& it : pos.position_of_legs)
      legs_pos.push_back(RoboLeg::Position(it.x, it.y, it.z));

    Hexapod::JntNames jnt_names = hexapod_.getJntNames();
    Hexapod::Angles angs;

    sensor_msgs::JointState jnt;
    jnt.header.stamp = ros::Time::now();

    for (std::size_t n = 0; n != legs_pos.size(); ++n)
    {
      try
      {
        hexapod_.getAnglesForSingleLeg(Hexapod::LegType(n),
                                       legs_pos[n], angs);
      }
      catch (std::logic_error e)
      {
        ROS_ERROR_STREAM(e.what());
        return;
      }
    }

    for (std::size_t i = 0; i != angs.size(); ++i)
      for (std::size_t j = 0; j != angs[i].size(); ++j)
      {
        jnt.name.push_back(jnt_names[i][j]);
        jnt.position.push_back(angs[i][j]);
      }
    jnt_pub_.publish(jnt);

    for (auto n : jnt.name)
      ROS_INFO_STREAM(" " << n << " ");
    for (auto p : jnt.position)
      ROS_INFO_STREAM(" " << p << " ");
  }

  ros::NodeHandle n_;
  ros::Publisher jnt_pub_;
  ros::Subscriber pos_sub_;
  Hexapod hexapod_;
};

int main(int argc, char** argv)
{
  sleep(2); // for debug purpose
  ros::init(argc, argv, "goliath_kinematics");

  if (argc != 2)
  {
    ROS_ERROR_STREAM("Need a URDF file as argument.");
    return -1;
  }

  ros::NodeHandle node;
  GoliathLocomotion goliath_locomotion(argv[1], node);

  ros::spin();
  return 0;
}
