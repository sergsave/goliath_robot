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

// class describes the insect-like six leg's robot
// class have instruments for calculating inverse and forward
// kinematics
class BodyKinematics
{
public:
  BodyKinematics() {}
  BodyKinematics(const urdf::Model&);

  // Enum for leg's side and location
  // left forward, left middle, left rear
  // right forward, right middle, right rear
  enum LegType
  {
    LF,
    LM,
    LR,
    RF,
    RM,
    RR,
    NUMBER_OF_LEGS
  };

  struct StanceOfLeg
  {
    StanceOfLeg() : rot_offset(0) {}
    LegKinematics::LegPos pos;
    double rot_offset;
  };

  typedef std::array<StanceOfLeg, NUMBER_OF_LEGS> LegsStance;
  typedef urdf::Pose BodyPose;

  // methods for calculating
  void calculateJntAngles(const BodyPose&,
                          trajectory_msgs::JointTrajectoryPoint&);
  void calculateJntAngles(const BodyPose&, const LegsStance&,
                          trajectory_msgs::JointTrajectoryPoint&);

  void getLegsJntName(std::vector<std::string>&);

  LegsStance getDefaultLegsStance();
  double getClearance();

private:
  typedef std::array<LegKinematics::LegPos, NUMBER_OF_LEGS> LegsPos;

  void calculateJntAngles(const LegsPos&,
                          trajectory_msgs::JointTrajectoryPoint&);

  // urdf::Vector3 doesnt have "-" operator
  urdf::Vector3 invVec(urdf::Vector3 vec)
  {
    return urdf::Vector3(-vec.x, -vec.y, -vec.z);
  }

  static const std::array<std::string, NUMBER_OF_LEGS> LEG_NAMES;
  static const std::string LEG_ROOT_JNT_BASE_NAME;
  std::array<LegKinematics, NUMBER_OF_LEGS> legs_;
  LegsPos legs_origin_;
};
#endif /* BODY_KINEMATICS_H */
