/*
 * hexa_leg.h
 *
 *  Created on: 06 янв. 2018 г.
 *      Author: sergey
 */

#ifndef BODY_KINEMATICS_H
#define BODY_KINEMATICS_H

#include <string>
#include <array>
#include "leg_kinematics.h"
#include "goliath_msgs/BodyPose.h"
#include "goliath_msgs/LegsPosition.h"
#include "sensor_msgs/JointState.h"

// class describes the insect-like six leg's robot
// class have instruments for calculating inverse and forward
// kinematics
class BodyKinematics
{
public:
  BodyKinematics() {}
  BodyKinematics(const urdf::Model&);

  // methods for calculating
  void calculateJntAngles(const goliath_msgs::LegsPosition&,
                          sensor_msgs::JointState&);
  void calculateJntAngles(const goliath_msgs::BodyPose&,
                          sensor_msgs::JointState&);
  void calculateJntAngles(const goliath_msgs::BodyPose&,
                          const goliath_msgs::LegsPosition&,
                          sensor_msgs::JointState&);

  double getClearance();
  goliath_msgs::LegsPosition getDefaultLegsPos();

private:
  // Enum for leg's side and location
  // left forward, left middle, left rear
  // right forward, right middle, right rear
  enum LegType
  {
    LF = 0,
    LM,
    LR,
    RF,
    RM,
    RR,
    NUMBER_OF_LEGS
  };

  static const std::array<std::string, NUMBER_OF_LEGS> LEG_NAMES;
  static const std::string LEG_ROOT_JNT_BASE_NAME;
  std::array<LegKinematics, NUMBER_OF_LEGS> legs_;
  std::array<geometry_msgs::Point32, NUMBER_OF_LEGS> legs_origin_;
};
#endif /* BODY_KINEMATICS_H */
