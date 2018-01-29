/*
 * hexa_leg.cpp
 *
 *  Created on: 06 янв. 2018 г.
 *      Author: sergey
 */
#include "hexapod.h"
#include <stdexcept>

using std::string;
using std::array;
using std::invalid_argument;
using std::logic_error;

const array<string, Hexapod::NUMBER_OF_LEGS> Hexapod::LEG_NAMES = {
    "lf", "lm", "lr", "rf", "rm", "rr",
};


Hexapod::Hexapod(const urdf::Model& model)
{
  //Range based loop
  for (auto& elem : legs_)
  {
    try
    {
      //init each leg
      elem = RoboLeg(LEG_NAMES[&elem - legs_.begin()], model);
    }
    catch (invalid_argument e)
    {
      //if urdf-model is not valid
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
