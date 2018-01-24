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

const array<string, HexaLeg::NUMBER_OF_SEGMENTS + 1> HexaLeg::JNT_BASE_NAMES = {
    "coxa_joint", "femur_joint", "tibia_joint", "tarsus_joint"};

HexaLeg::HexaLeg(const string& leg_prefix, const Model& model)
{
  array<JointConstSharedPtr, NUMBER_OF_SEGMENTS + 1> jnt;
  jnt[LegEnd] = model.getJoint(leg_prefix + JNT_BASE_NAMES[LegEnd]);

  for (std::size_t i = Coxa; i != segs_.size(); ++i)
  {
    segs_[i].name = leg_prefix + JNT_BASE_NAMES[i];

    if ((jnt[i] = model.getJoint(segs_[i].name)) == NULL ||
        jnt[i]->limits == NULL)
      throw("Not a valid URDF-model!");
    else
    {
      segs_[i].max_angle_ = jnt[i]->limits->upper;
      segs_[i].min_angle_ = jnt[i]->limits->lower;
    }
  }

  segs_[Coxa].length_ =
      fabs(jnt[Femur]->parent_to_joint_origin_transform.position.x);
  segs_[Femur].length_ =
      fabs(jnt[Tibia]->parent_to_joint_origin_transform.position.x);
  segs_[Tibia].length_ =
      fabs(jnt[LegEnd]->parent_to_joint_origin_transform.position.x);

  double roll, pitch, yaw;
  jnt[Coxa]->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch, yaw);
  segs_[Coxa].init_angle_ = yaw;
  jnt[Femur]->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch,
                                                               yaw);
  segs_[Femur].init_angle_ = pitch;
  jnt[Tibia]->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch,
                                                               yaw);
  segs_[Tibia].init_angle_ = pitch;
}

HexaLeg::IKResult HexaLeg::getAnglesIK(const Position& pos, Angles& angs)
{
  Angles temp;
  temp[Coxa] = atan2(pos.y_, pos.x_) - segs_[Coxa].init_angle_;
  // round up the angle
  temp[Coxa] +=
      temp[Coxa] > M_PI ? -2 * M_PI : (temp[Coxa] < -M_PI ? 2 * M_PI : 0);

  // additional angles and length for calculations
  // xy - coordinate of point on the leg axis
  // s - distance beetween root of the leg and point
  double xy, s;
  double alpha, beta;

  xy = hypot(pos.x_, pos.y_) - segs_[Coxa].length_;
  alpha = atan2(xy, pos.z_);
  s = hypot(xy, pos.z_);
  beta =
      acos((pow(segs_[Femur].length_, 2) + pow(s, 2) - pow(segs_[Tibia].length_, 2)) /
           (2 * s * segs_[Femur].length_));
  temp[Femur] = M_PI / 2 - (alpha + beta) - segs_[Femur].init_angle_;
  temp[Tibia] =
      -segs_[Tibia].init_angle_ -
      acos((pow(segs_[Femur].length_, 2) + pow(segs_[Tibia].length_, 2) - pow(s, 2)) /
           (2 * segs_[Tibia].length_ * segs_[Femur].length_));

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
  for (std::size_t i = Coxa; i != angs.size(); ++i)
    if (!segs_[i].checkAngle(angs[i]))
      return false;
  return true;
}

HexaLeg::JntNames HexaLeg::getJntNames()
{
  JntNames ret;
  std::size_t i;
  for (i = Coxa; i != segs_.size(); ++i)
    ret[i] = segs_[i].name;
  return ret;
}
