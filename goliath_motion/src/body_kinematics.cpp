/*
 * hexa_leg.cpp
 *
 *  Created on: 06 янв. 2018 г.
 *      Author: sergey
 */
#include "body_kinematics.h"
#include <stdexcept>

using std::string;
using std::array;
using std::invalid_argument;
using std::logic_error;

const array<string, BodyKinematics::NUMBER_OF_LEGS> BodyKinematics::LEG_NAMES =
    {
     "lf", "lm", "lr", "rf", "rm", "rr",
};
const string BodyKinematics::LEG_ROOT_JNT_BASE_NAME("_coxa_joint");

BodyKinematics::BodyKinematics(const urdf::Model& model)
{
  // for clearance checking
  double prev_clearance = 0;

  for (std::size_t l = LF; l != legs_.size(); l++)
  {
    // get a origins for leg
    urdf::JointConstSharedPtr joint =
        model.getJoint(LEG_NAMES[l] + LEG_ROOT_JNT_BASE_NAME);

    if (joint == NULL)
      throw invalid_argument("Not a valid URDF-model in " + LEG_NAMES[l] +
                             " leg's root");

    legs_origin_[l].x = joint->parent_to_joint_origin_transform.position.x;
    legs_origin_[l].y = joint->parent_to_joint_origin_transform.position.y;
    legs_origin_[l].z = joint->parent_to_joint_origin_transform.position.z;

    // init each leg
    try
    {
      legs_[l] = LegKinematics(LEG_NAMES[l], model);
    }
    catch (invalid_argument e)
    {
      // if urdf-model is not valid
      ROS_ERROR_STREAM(e.what());
    }

    // check clearance in model
    double clearance = legs_[l].getDefaultPos().z;

    if (l != LF && clearance != prev_clearance)
      throw logic_error("Unequal clearance for all legs");

    prev_clearance = clearance;
  }
}

BodyKinematics::LegsStance BodyKinematics::getDefaultLegsStance()
{
  LegsStance ret;

  for (std::size_t l = LF; l != legs_.size(); ++l)
    ret[l].pos = legs_[l].getDefaultPos();

  return ret;
}


void BodyKinematics::calculateJntAngles(
    const LegsPos& legs_pos, trajectory_msgs::JointTrajectoryPoint& jnt)
{
  // just call IK method for each leg
  for (std::size_t l = LF; l != legs_.size(); ++l)
  {
    if (legs_[l].calculateJntAngles(legs_pos[l], jnt) ==
        LegKinematics::ERROR)
      throw logic_error("Inverse kinematics error in " + LEG_NAMES[l] +
                        " leg!");
  }
}

// calculate joint angles with default legs stance
void BodyKinematics::calculateJntAngles(
    const BodyPose& pose, trajectory_msgs::JointTrajectoryPoint& jnt)
{
  LegsStance def_l_st = getDefaultLegsStance();

  calculateJntAngles(pose, def_l_st, jnt);
}

void BodyKinematics::calculateJntAngles(
    const BodyPose& body_pose, const LegsStance& legs_st,
    trajectory_msgs::JointTrajectoryPoint& jnt)
{
  LegsPos fin_positions;

  // create rotation matrix for legs, which inverse to body rotation
  urdf::Rotation rotation;
  double roll, pitch, yaw;
  body_pose.rotation.getRPY(roll, pitch, yaw);

  for (std::size_t l = LF; l != legs_.size(); ++l)
  {
    // vector from center of rotation to the end of leg
    LegKinematics::LegPos leg_end =
        legs_origin_[l] + legs_st[l].pos + invVec(body_pose.position);

    // Rotate leg_end vector in the direction opposite to the target.
    // So we get the coordinates of the default leg end pos in new
    // rotated (rot) system of coordinates (frame)
    // This is example for target yaw = -45 degree.
    //
    //         y/\   *rot_x
    //  rot_y   |   *
    //  *       |  *
    //       *  | *
    //          0------>x
    //
    rotation.setFromRPY(-roll, -pitch, -yaw + legs_st[l].rot_offset);
    LegKinematics::LegPos leg_end_in_rot_frame = rotation * leg_end;

    // When body turns, leg should stand, coordinates of leg_end should
    // be default. But with body turns a frame, and for calculation should
    // be used leg_end_in_rot_frame
    fin_positions[l] = leg_end_in_rot_frame + invVec(legs_origin_[l]);
  }

  calculateJntAngles(fin_positions, jnt);
}

// clearances for all legs are equal, so we can just get clearance for
// first leg
double BodyKinematics::getClearance() { return legs_[LF].getDefaultPos().z; }

void BodyKinematics::getLegsJntName(std::vector<std::string>& names)
{
  for (auto& l : legs_)
    l.getJntNames(names);
}
