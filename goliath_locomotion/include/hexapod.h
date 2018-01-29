/*
 * hexa_leg.h
 *
 *  Created on: 06 янв. 2018 г.
 *      Author: sergey
 */

#ifndef HEXAPOD_H_
#define HEXAPOD_H_

#include <string>
#include <array>
#include "robo_leg.h"

//class describes the insect-like six leg's robot
//class have instruments for calculating inverse and forward
//kinematics
class Hexapod
{
public:
  Hexapod() {}
  Hexapod(const urdf::Model&);

  //Enum for leg's side and location
  //left forward, left middle, left rear
  //right forward, right middle, right rear
  enum LegType
  {
    LF = 0,
    LM,
    LR,
    RF,
    RM,
    RR,
    NUMBER_OF_LEGS
  };

  //method's for calculating
  typedef std::array<RoboLeg::Angles, NUMBER_OF_LEGS> Angles;
  typedef std::array<RoboLeg::JntNames, NUMBER_OF_LEGS> JntNames;

  void getAnglesForSingleLeg(LegType, const RoboLeg::Position&, Angles &angs);
  JntNames getJntNames();

private:
  static const std::array<std::string, NUMBER_OF_LEGS> LEG_NAMES;
  std::array<RoboLeg, NUMBER_OF_LEGS> legs_;
};
#endif /* HEXAPOD_H_ */
