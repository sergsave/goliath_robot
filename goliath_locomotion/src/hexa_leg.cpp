/*
 * hexa_leg.cpp
 *
 *  Created on: 06 янв. 2018 г.
 *      Author: sergey
 */
#include "hexa_leg.h"
#include <iostream>
#include <cmath>

using urdf::Model;
using urdf::JointConstSharedPtr;
using std::string;
using std::array;

static const int NUMBER_OF_SEGMENTS = 3;
const string HexaLeg::COXA_J_NAME = "coxa_joint";
const string HexaLeg::FEMUR_J_NAME = "femur_joint";
const string HexaLeg::TIBIA_J_NAME = "tibia_joint";
const string HexaLeg::TARSUS_J_NAME = "tarsus_joint";

HexaLeg::HexaLeg(const string& leg_prefix, const Model& model)
{
  segs_[Coxa].name = leg_prefix + COXA_J_NAME;
  JointConstSharedPtr coxa_j = model.getJoint(segs_[Coxa].name);
  segs_[Femur] = leg_prefix + FEMUR_J_NAME;
  JointConstSharedPtr femur_j = model.getJoint(segs_[Femur].name);
  tibia_.name = leg_prefix + TIBIA_J_NAME;
  JointConstSharedPtr tibia_j = model.getJoint(tibia_.name);
  string j_name = leg_prefix + TARSUS_J_NAME;
  JointConstSharedPtr tarsus_j = model.getJoint(j_name);

  if (!coxa_j || !femur_j || !tibia_j || !tarsus_j || !coxa_j->limits ||
      !femur_j->limits || !tibia_j->limits)
  {
    throw("Not a valid URDF-model!");
  }

  coxa_.length_ = fabs(femur_j->parent_to_joint_origin_transform.position.x);
  femur_.length_ = fabs(tibia_j->parent_to_joint_origin_transform.position.x);
  tibia_.length_ = fabs(tarsus_j->parent_to_joint_origin_transform.position.x);

  double roll, pitch, yaw;
  coxa_j->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch, yaw);
  coxa_.init_angle_ = yaw;
  femur_j->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch, yaw);
  femur_.init_angle_ = pitch;
  tibia_j->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch, yaw);
  tibia_.init_angle_ = pitch;

  coxa_.max_angle_ = coxa_j->limits->upper;
  coxa_.min_angle_ = coxa_j->limits->lower;
  femur_.max_angle_ = femur_j->limits->upper;
  femur_.min_angle_ = femur_j->limits->lower;
  tibia_.max_angle_ = tibia_j->limits->upper;
  tibia_.min_angle_ = tibia_j->limits->lower;
}

HexaLeg::IKResult HexaLeg::getAnglesIK(const Position& pos, Angles& angs)
{
  Angles temp;
  temp.coxa_ = atan2(pos.y_, pos.x_) - coxa_.init_angle_;
  // round up the angle
  temp.coxa_ +=
      temp.coxa_ > M_PI ? -2 * M_PI : (temp.coxa_ < -M_PI ? 2 * M_PI : 0);

  // additional angles and length for calculations
  // xy - coordinate of point on the leg axis
  // s - distance beetween root of the leg and point
  double xy, s;
  double alpha, beta;

  xy = hypot(pos.x_, pos.y_) - coxa_.length_;
  alpha = atan2(xy, pos.z_);
  s = hypot(xy, pos.z_);
  beta = acos((pow(femur_.length_, 2) + pow(s, 2) - pow(tibia_.length_, 2)) /
              (2 * s * femur_.length_));
  temp.femur_ = M_PI / 2 - (alpha + beta) - femur_.init_angle_;
  temp.tibia_ =
      -tibia_.init_angle_ -
      acos((pow(femur_.length_, 2) + pow(tibia_.length_, 2) - pow(s, 2)) /
           (2 * tibia_.length_ * femur_.length_));

  if (checkAngles(temp))
  {
    angs = temp;
    return OK;
  }
  else
    return Error;
}

bool HexaLeg::checkAngles(Angles& angs)
{
  return coxa_.checkAngle(angs.coxa_) && femur_.checkAngle(angs.femur_) &&
         tibia_.checkAngle(angs.tibia_);
};

HexaLeg::JntNames HexaLeg::getJntNames()
{
  JntNames ret;
  ret.coxa_ = coxa_.name;
  ret.femur_ = femur_.name;
  ret.tibia_ = tibia_.name;
  return ret;
}
