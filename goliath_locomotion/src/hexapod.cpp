/*
 * hexa_leg.cpp
 *
 *  Created on: 06 янв. 2018 г.
 *      Author: sergey
 */
#include "hexapod.h"
#include <stdexcept>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

using std::string;
using std::array;
using std::invalid_argument;
using std::logic_error;

const array<string, Hexapod::NUMBER_OF_LEGS> Hexapod::LEG_NAMES = {
    "lf", "lm", "lr", "rf", "rm", "rr",
};

Hexapod::Hexapod(const urdf::Model& model)
{
  // Range based loop
  for (auto& elem : legs_)
  {
    try
    {
      // init each leg
      elem = RoboLeg(LEG_NAMES[&elem - legs_.begin()], model);
    }
    catch (invalid_argument e)
    {
      // if urdf-model is not valid
      ROS_ERROR_STREAM(e.what());
    }
  }
}

Hexapod::JntNames Hexapod::getJntNames()
{
  JntNames ret;
  for (auto& elem : ret)
    elem = legs_[&elem - ret.begin()].getJntNames();
  return ret;
}

void Hexapod::getAnglesForSingleLeg(Hexapod::LegType type,
                                    const RoboLeg::Position& pos,
                                    Hexapod::Angles& angs)
{
  RoboLeg::Angles temp;

  if (legs_[type].getAnglesIK(pos, temp) == RoboLeg::OK)
    angs[type] = temp;
  else
    throw logic_error("Inverse kinematics error!");
}

void Hexapod::getAnglesFromBodyPose(Hexapod::LegType type,
                                   const goliath_msgs::BodyPose& pose,
                                   Hexapod::Angles& angs,
                                   RoboLeg::Position& debug)
{
  RoboLeg::Position origin = legs_[type].origin();
  RoboLeg::Position default_pos = legs_[type].getDefault();
  boost::numeric::ublas::vector<double> leg_end(3);

  leg_end(0) = origin.x + default_pos.x - pose.position.x;
  leg_end(1) = origin.y + default_pos.y - pose.position.y;
  leg_end(2) = origin.z + default_pos.z - pose.position.z;

  double a = cos(-pose.roll);
  double b = sin(-pose.roll);
  double c = cos(-pose.pitch);
  double d = sin(-pose.pitch);
  double e = cos(-pose.yaw);
  double f = sin(-pose.yaw);

  boost::numeric::ublas::matrix<double> mat_x(3, 3);
  mat_x(0, 0) = 1, mat_x(0, 1) = 0, mat_x(0, 2) = 0;
  mat_x(1, 0) = 0, mat_x(1, 1) = a, mat_x(1, 2) = -b;
  mat_x(2, 0) = 0, mat_x(2, 1) = b, mat_x(2, 2) = a;
  boost::numeric::ublas::matrix<double> mat_y(3, 3);
  mat_y(0, 0) = c, mat_y(0, 1) = 0, mat_y(0, 2) = d;
  mat_y(1, 0) = 0, mat_y(1, 1) = 1, mat_y(1, 2) = 0;
  mat_y(2, 0) = -d, mat_y(2, 1) = 0, mat_y(2, 2) = c;
  boost::numeric::ublas::matrix<double> mat_z(3, 3);
  mat_z(0, 0) = e, mat_z(0, 1) = -f, mat_z(0, 2) = 0;
  mat_z(1, 0) = f, mat_z(1, 1) = e, mat_z(1, 2) = 0;
  mat_z(2, 0) = 0, mat_z(2, 1) = 0, mat_z(2, 2) = 1;

  boost::numeric::ublas::matrix<double> mat_1 =
      boost::numeric::ublas::prod(mat_z, mat_y);
  boost::numeric::ublas::matrix<double> rotation =
      boost::numeric::ublas::prod(mat_1, mat_x);

  boost::numeric::ublas::vector<double> leg_end_in_rot_frame =
      boost::numeric::ublas::prod(rotation, leg_end);


  RoboLeg::Angles temp;
  RoboLeg::Position fin_pos;
  fin_pos.x = leg_end_in_rot_frame(0) - origin.x;
  fin_pos.y = leg_end_in_rot_frame(1) - origin.y;
  fin_pos.z = leg_end_in_rot_frame(2) - origin.z;

  if (legs_[type].getAnglesIK(fin_pos, temp) == RoboLeg::OK)
    angs[type] = temp;
  else
    // ROS_ERROR_STREAM("Inverse kinematics error!");
    throw logic_error("Inverse kinematics error!");
}
