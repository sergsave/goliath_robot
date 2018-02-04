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

void Hexapod::getAnglesFromBodyPos(Hexapod::LegType type,
                                   const goliath_msgs::Body_pos& pos,
                                   Hexapod::Angles& angs)
{
  RoboLeg::Position origin = legs_[type].origin();
  RoboLeg::Position default_pos = legs_[type].getDefault();
  boost::numeric::ublas::vector<double> vec_origin(3);

  vec_origin(0) = origin.x; //+ default_pos.x;
  vec_origin(1) = origin.y; //+ default_pos.y;
  vec_origin(2) = origin.z; //+ default_pos.z;

  double a = cos(pos.roll);
  double b = sin(pos.roll);
  double c = cos(pos.pitch);
  double d = sin(pos.pitch);
  double e = cos(pos.yaw);
  double f = sin(pos.yaw);

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

  boost::numeric::ublas::vector<double> vec_origin_rot =
      boost::numeric::ublas::prod(rotation, vec_origin);
  if (type == LF)
    ROS_ERROR_STREAM(" rotated origin: " << vec_origin_rot);

  boost::numeric::ublas::vector<double> vec(3);
  vec(0) = (origin.x + default_pos.x) - vec_origin_rot(0);
  vec(1) = (origin.y + default_pos.y) - vec_origin_rot(1);
  vec(2) = (origin.z + default_pos.z) - vec_origin_rot(2);

  if (type == LF)
    ROS_ERROR_STREAM(" leg end coord - rotated origin coord " << vec);

  a = cos(-pos.roll);
  b = sin(-pos.roll);
  c = cos(-pos.pitch);
  d = sin(-pos.pitch);
  e = cos(-pos.yaw);
  f = sin(-pos.yaw);

  mat_x(0, 0) = 1, mat_x(0, 1) = 0, mat_x(0, 2) = 0;
  mat_x(1, 0) = 0, mat_x(1, 1) = a, mat_x(1, 2) = -b;
  mat_x(2, 0) = 0, mat_x(2, 1) = b, mat_x(2, 2) = a;

  mat_y(0, 0) = c, mat_y(0, 1) = 0, mat_y(0, 2) = d;
  mat_y(1, 0) = 0, mat_y(1, 1) = 1, mat_y(1, 2) = 0;
  mat_y(2, 0) = -d, mat_y(2, 1) = 0, mat_y(2, 2) = c;

  mat_z(0, 0) = e, mat_z(0, 1) = -f, mat_z(0, 2) = 0;
  mat_z(1, 0) = f, mat_z(1, 1) = e, mat_z(1, 2) = 0;
  mat_z(2, 0) = 0, mat_z(2, 1) = 0, mat_z(2, 2) = 1;

  mat_1 = boost::numeric::ublas::prod(mat_z, mat_y);
  rotation = boost::numeric::ublas::prod(mat_1, mat_x);

  boost::numeric::ublas::vector<double> vec_rot =
      boost::numeric::ublas::prod(rotation, vec);

  if (type == LF)
    ROS_ERROR_STREAM(" result " << vec_rot);

  RoboLeg::Angles temp;
  RoboLeg::Position fin_pos;
  fin_pos.x = vec_rot(0);//default_pos.x - vec_rot(0);
  fin_pos.y = vec_rot(1);//default_pos.y - vec_rot(1);
  fin_pos.z = vec_rot(2);//default_pos.z - vec_rot(2);

  if (legs_[type].getAnglesIK(fin_pos, temp) == RoboLeg::OK)
    angs[type] = temp;
  else
    // ROS_ERROR_STREAM("Inverse kinematics error!");
    throw logic_error("Inverse kinematics error!");
}
