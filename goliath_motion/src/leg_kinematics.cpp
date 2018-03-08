/*
 * hexa_leg.cpp
 *
 *  Created on: 06 янв. 2018 г.
 *      Author: sergey
 */

#include "leg_kinematics.h"
#include <cmath>
#include <stdexcept>

using urdf::Model;
using urdf::JointConstSharedPtr;
using std::string;
using std::array;
using std::invalid_argument;
using sensor_msgs::JointState;
using geometry_msgs::Point32;

const array<string, LegKinematics::NUMBER_OF_SEGMENTS + 1>
    LegKinematics::JNT_BASE_NAMES = {"coxa_joint", "femur_joint", "tibia_joint",
                                     "tarsus_joint"};

LegKinematics::LegKinematics(const string& leg_prefix, const Model& model)
{
  array<JointConstSharedPtr, NUMBER_OF_SEGMENTS + 1> joints;

  // create joint for leg end (fixed joint) from model
  if ((joints[NUMBER_OF_SEGMENTS] = model.getJoint(
           leg_prefix + '_' + JNT_BASE_NAMES[NUMBER_OF_SEGMENTS])) == NULL)
  {
    throw invalid_argument("Not a valid URDF-model in " + leg_prefix + " leg");
  }

  // create all other joints from model
  for (std::size_t i = COXA; i != segs_.size(); ++i)
  {
    segs_[i].name = leg_prefix + '_' + JNT_BASE_NAMES[i];

    if ((joints[i] = model.getJoint(segs_[i].name)) == NULL ||
        joints[i]->limits == NULL)
    {
      throw invalid_argument("Not a valid URDF-model in " + leg_prefix +
                             " leg");
    }
    else
    {
      segs_[i].max_angle = joints[i]->limits->upper;
      segs_[i].min_angle = joints[i]->limits->lower;
    }
  }

  // set joint's parameters from urdf-model
  segs_[COXA].length =
      fabs(joints[FEMUR]->parent_to_joint_origin_transform.position.x);
  segs_[FEMUR].length =
      fabs(joints[TIBIA]->parent_to_joint_origin_transform.position.x);
  segs_[TIBIA].length = fabs(
      joints[NUMBER_OF_SEGMENTS]->parent_to_joint_origin_transform.position.x);

  //set default RPY angles
  double roll, pitch, yaw;
  joints[COXA]->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch,
                                                                 yaw);
  segs_[COXA].init_angle = yaw;
  joints[FEMUR]->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch,
                                                                  yaw);
  segs_[FEMUR].init_angle = pitch;
  joints[TIBIA]->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch,
                                                                  yaw);
  segs_[TIBIA].init_angle = pitch;

  def_pos_ = calculateDefaultPos();
}

LegKinematics::IKResult
LegKinematics::calculateJntAngles(const Point32& pos, JointState& jnt_angles)
{
  Angles temp;
  temp[COXA] = atan2(pos.y, pos.x) - segs_[COXA].init_angle;

  // round up the angle
  temp[COXA] +=
      temp[COXA] > M_PI ? -2 * M_PI : (temp[COXA] < -M_PI ? 2 * M_PI : 0);

  // additional angles and length for calculations,
  // xy - coordinate of point on the leg axis,
  // s - line segment beetween root(0) of the leg and point (S).
  // alpha - angle beetween z axis and s
  // beta - angle beetween femur(fem) and s
  //
  //
  //          /\
  // fem_l-> /  \ <-tib_l
  //        /    \     xy
  // ------0      \ ---*------> XY axis
  //       |       \
  //       |        \
  //       |         \
  //       |          \
  //       \/ -Z axis  *S
  double xy, s;
  double alpha, beta;
  double& fem_l = segs_[FEMUR].length;
  double& tib_l = segs_[TIBIA].length;

  xy = hypot(pos.x, pos.y) - segs_[COXA].length;

  if (xy > tib_l + fem_l)
    return ERROR;

  alpha = atan2(xy, -pos.z);
  s = hypot(xy, -pos.z);

  // Use law of cosines
  beta = acos((pow(fem_l, 2) + pow(s, 2) - pow(tib_l, 2)) / (2 * s * fem_l));
  temp[FEMUR] = M_PI / 2 - (alpha + beta) - segs_[FEMUR].init_angle;

  // Calculate tibia angle with law of cosines too
  temp[TIBIA] =
      -segs_[TIBIA].init_angle -
      acos((pow(fem_l, 2) + pow(tib_l, 2) - pow(s, 2)) / (2 * tib_l * fem_l));

  // protection from robot's legs crash
  if (checkAngles(temp))
  {
    jnt_angles.position.insert(jnt_angles.position.end(), temp.begin(),
                               temp.end());
    for (auto& elem : segs_)
      jnt_angles.name.push_back(elem.name);

    return OK;
  }
  else
    return ERROR;
}

bool LegKinematics::checkAngles(Angles& angs) const
{
  for (std::size_t i = COXA; i != angs.size(); ++i)
    if (!segs_[i].checkAngle(angs[i]))
      return false;
  return true;
}

Point32 LegKinematics::calculateDefaultPos()
{
  Point32 ret;

  // projection of segments to xy axis (see comments in calculateJntAngles
  // method)
  double coxa_xy = segs_[COXA].length;
  double femur_xy = segs_[FEMUR].length * cos(segs_[FEMUR].init_angle);
  double tibia_xy = segs_[TIBIA].length *
                    -cos(segs_[FEMUR].init_angle + segs_[TIBIA].init_angle);
  // projection of segments to z axis
  double coxa_z = 0;
  double femur_z = segs_[FEMUR].length * sin(segs_[FEMUR].init_angle);
  double tibia_z = segs_[TIBIA].length *
                   -sin(segs_[FEMUR].init_angle + segs_[TIBIA].init_angle);

  double xy = coxa_xy + femur_xy + tibia_xy;

  ret.z = -(coxa_z + femur_z + tibia_z);
  ret.x = xy * cos(segs_[COXA].init_angle);
  ret.y = xy * sin(segs_[COXA].init_angle);

  return ret;
}
