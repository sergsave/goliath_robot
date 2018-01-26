/*
 * hexa_leg.cpp
 *
 *  Created on: 06 янв. 2018 г.
 *      Author: sergey
 */
#include "hexapod.h"
#include <iostream>
#include <cmath>
#include <stdexcept>

using urdf::Model;
using urdf::JointConstSharedPtr;
using std::string;
using std::array;
using std::invalid_argument;
using std::logic_error;

const array<string, Hexapod::NUMBER_OF_LEGS> Hexapod::LEG_NAMES = {
    "lf", "lm", "lr", "rf", "rm", "rr",
};

Hexapod::Hexapod(const Model& model)
{
  // Let's try to use range based loop
  for (auto& elem : legs_)
  {
    try
    {
      elem = RoboLeg(LEG_NAMES[&elem - legs_.begin()], model);
    }
    catch (invalid_argument e)
    {
      ROS_ERROR_STREAM(e.what());
    }
  }
}

Hexapod::Angles Hexapod::getAnglesForSingleLeg(Hexapod::LegType type,
                                               const RoboLeg::Position& pos)
{
  Hexapod::Angles ret;
  RoboLeg::Angles temp;

  if (legs_[type].getAnglesIK(pos, temp) == RoboLeg::OK)
    ret[type] = temp;
  else
    throw logic_error("Inverse kinematics error!");

  return ret;
}

Hexapod::JntNames Hexapod::getJntNames()
{
  JntNames ret;
  for (auto& elem : ret)
    elem = legs_[&elem - ret.begin()].getJntNames();
  return ret;
}

const array<string, RoboLeg::NUMBER_OF_SEGMENTS + 1> RoboLeg::JNT_BASE_NAMES = {
    "coxa_joint", "femur_joint", "tibia_joint", "tarsus_joint"};

RoboLeg::RoboLeg(const string& leg_prefix, const Model& model)
{
  array<JointConstSharedPtr, NUMBER_OF_SEGMENTS + 1> jnt;

  // create joint for leg end (fixed joint)
  if ((jnt[NUMBER_OF_SEGMENTS] = model.getJoint(
           leg_prefix + '_' + JNT_BASE_NAMES[NUMBER_OF_SEGMENTS])) == NULL)
    throw invalid_argument("Not a valid URDF-model in " + leg_prefix + " leg");

  // create all other joints
  for (std::size_t i = COXA; i != segs_.size(); ++i)
  {
    segs_[i].name = leg_prefix + '_' + JNT_BASE_NAMES[i];

    if ((jnt[i] = model.getJoint(segs_[i].name)) == NULL ||
        jnt[i]->limits == NULL)
      throw invalid_argument("Not a valid URDF-model in " + leg_prefix +
                             " leg");
    else
    {
      segs_[i].max_angle = jnt[i]->limits->upper;
      segs_[i].min_angle = jnt[i]->limits->lower;
    }
  }

  segs_[COXA].length =
      fabs(jnt[FEMUR]->parent_to_joint_origin_transform.position.x);
  segs_[FEMUR].length =
      fabs(jnt[TIBIA]->parent_to_joint_origin_transform.position.x);
  segs_[TIBIA].length = fabs(
      jnt[NUMBER_OF_SEGMENTS]->parent_to_joint_origin_transform.position.x);

  double roll, pitch, yaw;
  jnt[COXA]->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch, yaw);
  segs_[COXA].init_angle = yaw;
  jnt[FEMUR]->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch,
                                                               yaw);
  segs_[FEMUR].init_angle = pitch;
  jnt[TIBIA]->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch,
                                                               yaw);
  segs_[TIBIA].init_angle = pitch;
}

RoboLeg::IKResult RoboLeg::getAnglesIK(const Position& pos, Angles& angs)
{
  Angles temp;
  temp[COXA] = atan2(pos.x, pos.x) - segs_[COXA].init_angle;
  // round up the angle
  temp[COXA] +=
      temp[COXA] > M_PI ? -2 * M_PI : (temp[COXA] < -M_PI ? 2 * M_PI : 0);

  // additional angles and length for calculations
  // xy - coordinate of point on the leg axis
  // s - distance beetween root of the leg and point
  double xy, s;
  double alpha, beta;

  xy = hypot(pos.x, pos.x) - segs_[COXA].length;
  alpha = atan2(xy, pos.z);
  s = hypot(xy, pos.z);
  beta = acos(
      (pow(segs_[FEMUR].length, 2) + pow(s, 2) - pow(segs_[TIBIA].length, 2)) /
      (2 * s * segs_[FEMUR].length));
  temp[FEMUR] = M_PI / 2 - (alpha + beta) - segs_[FEMUR].init_angle;
  temp[TIBIA] = -segs_[TIBIA].init_angle -
                acos((pow(segs_[FEMUR].length, 2) +
                      pow(segs_[TIBIA].length, 2) - pow(s, 2)) /
                     (2 * segs_[TIBIA].length * segs_[FEMUR].length));

  if (checkAngles(temp))
  {
    angs = temp;
    return OK;
  }
  else
    return ERROR;
}

bool RoboLeg::checkAngles(Angles& angs) const
{
  for (std::size_t i = COXA; i != angs.size(); ++i)
    if (!segs_[i].checkAngle(angs[i]))
      return false;
  return true;
}

RoboLeg::JntNames RoboLeg::getJntNames()
{
  JntNames ret;
  for (std::size_t i = COXA; i != segs_.size(); ++i)
    ret[i] = segs_[i].name;

  return ret;
}
