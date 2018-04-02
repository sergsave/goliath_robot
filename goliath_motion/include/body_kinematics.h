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
// class have methods for calculating inverse and forward
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

  // Rotation offset is used when robot turns
  struct StanceOfLeg
  {
    StanceOfLeg() : rot_offset(0) {}
    LegKinematics::LegPos pos;
    double rot_offset;
  };

  typedef std::array<StanceOfLeg, NUMBER_OF_LEGS> LegsStance;
  typedef urdf::Pose BodyPose;

  // convert body pose and legs postitions (relatively legs roots)
  // in joint angles
  // this method put 18 angles in argument, so be careful with
  // non empty JointTrajectoryPoint
  void calculateJntAngles(const BodyPose&,
                          trajectory_msgs::JointTrajectoryPoint&);
  void calculateJntAngles(const BodyPose&, const LegsStance&,
                          trajectory_msgs::JointTrajectoryPoint&);

  // this method just put 18 names in argument
  // in the same order as calculateJntAngles methods
  // be careful with non empty vectors!
  void getLegsJntName(std::vector<std::string>&);

  // default coordinates of legs ends relatively
  // legs root
  LegsStance getDefaultLegsStance();

  // from center of the body to ground
  double getClearance();

private:
  static const std::array<std::string, NUMBER_OF_LEGS> LEG_NAMES;
  static const std::string LEG_ROOT_JNT_BASE_NAME;

  typedef std::array<LegKinematics::LegPos, NUMBER_OF_LEGS> LegsPos;

  void calculateJntAngles(const LegsPos&,
                          trajectory_msgs::JointTrajectoryPoint&);

  // urdf::Vector3 doesnt have "-" operator :(
#warning "is there any way to replace with operator-?"
  urdf::Vector3 invVec(urdf::Vector3 vec)
  {
    return urdf::Vector3(-vec.x, -vec.y, -vec.z);
  }

  std::array<LegKinematics, NUMBER_OF_LEGS> legs_;
  LegsPos legs_origin_;
};
#endif /* BODY_KINEMATICS_H */
