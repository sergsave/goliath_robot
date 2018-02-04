/*
 * hexa_leg.cpp
 *
 *  Created on: 06 янв. 2018 г.
 *      Author: sergey
 */
#include "robo_leg.h"
#include <cmath>
#include <stdexcept>

using urdf::Model;
using urdf::JointConstSharedPtr;
using std::string;
using std::array;
using std::invalid_argument;

const array<string, RoboLeg::NUMBER_OF_SEGMENTS + 1> RoboLeg::JNT_BASE_NAMES = {
    "coxa_joint", "femur_joint", "tibia_joint", "tarsus_joint"};

RoboLeg::RoboLeg(const string& leg_prefix, const Model& model)
{
  array<JointConstSharedPtr, NUMBER_OF_SEGMENTS + 1> jnt;

  // create joint for leg end (fixed joint) from model
  if ((jnt[NUMBER_OF_SEGMENTS] = model.getJoint(
           leg_prefix + '_' + JNT_BASE_NAMES[NUMBER_OF_SEGMENTS])) == NULL)
    throw invalid_argument("Not a valid URDF-model in " + leg_prefix + " leg");

  // create all other joints from model
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

  // set origin
  origin_.z = 0;
  origin_.x = jnt[COXA]->parent_to_joint_origin_transform.position.x;
  origin_.y = jnt[COXA]->parent_to_joint_origin_transform.position.y;

  // set joint's parameters from urdf-model
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
  temp[COXA] = atan2(pos.y, pos.x) - segs_[COXA].init_angle;

  // round up the angle
  temp[COXA] +=
      temp[COXA] > M_PI ? -2 * M_PI : (temp[COXA] < -M_PI ? 2 * M_PI : 0);

  // additional angles and length for calculations,
  // xy - coordinate of point on the leg axis,
  // s - distance beetween root of the leg and point.
  double xy, s;
  double alpha, beta;

  xy = hypot(pos.x, pos.y) - segs_[COXA].length;
  if(xy > segs_[TIBIA].length + segs_[FEMUR].length)
    return ERROR;

  alpha = atan2(xy, pos.z);
  s = hypot(xy, pos.z);

  // Use cos-theorem
  beta = acos(
      (pow(segs_[FEMUR].length, 2) + pow(s, 2) - pow(segs_[TIBIA].length, 2)) /
      (2 * s * segs_[FEMUR].length));
  temp[FEMUR] = M_PI / 2 - (alpha + beta) - segs_[FEMUR].init_angle;

  // calculate tibia angle with cos-theorem
  temp[TIBIA] = -segs_[TIBIA].init_angle -
                acos((pow(segs_[FEMUR].length, 2) +
                      pow(segs_[TIBIA].length, 2) - pow(s, 2)) /
                     (2 * segs_[TIBIA].length * segs_[FEMUR].length));

  // protection from robot's legs crash
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

RoboLeg::Position RoboLeg::getDefault()
{
  Position ret;
  double xy = segs_[COXA].length +
              segs_[FEMUR].length * cos(segs_[FEMUR].init_angle) +
              segs_[TIBIA].length *
                  -cos(segs_[FEMUR].init_angle + segs_[TIBIA].init_angle);
  ret.z = segs_[FEMUR].length * sin(segs_[FEMUR].init_angle) +
          segs_[TIBIA].length *
              -sin(segs_[FEMUR].init_angle + segs_[TIBIA].init_angle);

  ret.x = xy*cos(segs_[COXA].init_angle);
  ret.y = xy*sin(segs_[COXA].init_angle);
  return ret;
}
