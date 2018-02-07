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
  for (std::size_t i = LF; i != leg_origins_.size(); i++)
  {
    // get a origins for leg
    urdf::JointConstSharedPtr joint =
        model.getJoint(LEG_NAMES[i] + LEG_ROOT_JNT_BASE_NAME);

    if (joint == NULL)
      throw invalid_argument("Not a valid URDF-model in " + LEG_NAMES[i] +
                             " leg's root");

    leg_origins_[i].x = joint->parent_to_joint_origin_transform.position.x;
    leg_origins_[i].y = joint->parent_to_joint_origin_transform.position.y;
    leg_origins_[i].z = joint->parent_to_joint_origin_transform.position.z;

    // init each leg
    try
    {

      legs_[i] = LegKinematics(LEG_NAMES[i], model);
    }
    catch (invalid_argument e)
    {
      // if urdf-model is not valid
      ROS_ERROR_STREAM(e.what());
    }
  }
}

// calculate angles for controle single leg
void BodyKinematics::calculateJntAngles(
    const goliath_msgs::LegsPosition& leg_pos, sensor_msgs::JointState& jnt)
{
  // just call IK method for each leg
  for (std::size_t i = LF; i != legs_.size(); ++i)
  {
    if (legs_[i].calculateJntAngles(leg_pos.position_of_legs[i], jnt) ==
        LegKinematics::ERROR)
      throw logic_error("Inverse kinematics error in " + LEG_NAMES[i] + " leg!");
  }
}

// calculate angles for controle body
void BodyKinematics::calculateJntAngles(const goliath_msgs::BodyPose& pose,
                                        sensor_msgs::JointState& jnt)
{
  // use rotation matrix
  urdf::Rotation rotation;
  rotation.setFromRPY(-pose.roll, -pose.pitch, -pose.yaw);

  for (std::size_t i = LF; i != legs_.size(); ++i)
  {
    geometry_msgs::Point32 default_pos = legs_[i].calculateDefaultPos();

    // vector from center of rotation to the end of leg
    urdf::Vector3 leg_end;

    leg_end.x = leg_origins_[i].x + default_pos.x - pose.position.x;
    leg_end.y = leg_origins_[i].y + default_pos.y - pose.position.y;
    leg_end.z = leg_origins_[i].z + default_pos.z - pose.position.z;

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
    urdf::Vector3 leg_end_in_rot_frame = rotation * leg_end;

    // When body turns, leg should stand, coordinates of leg_end should
    // be default. But with body turns a frame, and for calculation should
    // be used leg_end_in_rot_frame
    geometry_msgs::Point32 fin_pos;
    fin_pos.x = leg_end_in_rot_frame.x - leg_origins_[i].x;
    fin_pos.y = leg_end_in_rot_frame.y - leg_origins_[i].y;
    fin_pos.z = leg_end_in_rot_frame.z - leg_origins_[i].z;

    if (legs_[i].calculateJntAngles(fin_pos, jnt) == LegKinematics::ERROR)
      throw logic_error("Inverse kinematics error in " + LEG_NAMES[i] + " leg!");
  }
}

double BodyKinematics::getClearance()
{
  double cl, prev_cl;

  for (std::size_t i = LF; i != legs_.size(); ++i)
  {
    cl = legs_[LF].calculateDefaultPos().z;

    if (i != LF && cl != prev_cl)
      throw logic_error("Unequal clearance for all legs");

    prev_cl = cl;
  }
  return cl;
}
