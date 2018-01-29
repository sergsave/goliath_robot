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

// this class contain motion control mechanism
// for Goliath - 6 leg's insect's like robot.
class GoliathLocomotion
{
public:
  GoliathLocomotion(ros::NodeHandle node) : n_(node)
  {
    urdf::Model model;

    //use parameter from ROS parameter server for create URDF-model.
    //if an xml robot's description is not added to server, and error
    //will occur
    if (!model.initParam("robot_description"))
    {
      ROS_FATAL_STREAM("Need a \"robot description\" parameter on server"
                                    << endl);
      exit(-1);
    }

    hexapod_ = Hexapod(model);

    //create public's publisher and subscriber
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

    //convert input Pose-message in RoboLeg::pos
    for (auto& it : pos.position_of_legs)
      legs_pos.push_back(RoboLeg::Position(it.x, it.y, it.z));

    Hexapod::JntNames jnt_names = hexapod_.getJntNames();
    Hexapod::Angles angs;

    sensor_msgs::JointState jnt;
    jnt.header.stamp = ros::Time::now();

    //calculate inverse kinematics for each leg
    for (std::size_t n = 0; n != legs_pos.size(); ++n)
    {
      try
      {
        hexapod_.getAnglesForSingleLeg(Hexapod::LegType(n), legs_pos[n], angs);
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

    // print all published data for debug - info
    for (std::size_t i = 0;  i != jnt.name.size(); ++i)
      ROS_INFO_STREAM(jnt.name[i]<< ' ' << jnt.position[i] << std::endl);

  }

  ros::NodeHandle n_;
  ros::Publisher jnt_pub_;
  ros::Subscriber pos_sub_;
  Hexapod hexapod_;
};

int main(int argc, char** argv)
{
  // for debug purpose
  sleep(2);
  // Let's start ROS!
  ros::init(argc, argv, "goliath_kinematics");

  ros::NodeHandle node;
  GoliathLocomotion goliath_locomotion(node);

  //wait
  ros::spin();
  return 0;
}
