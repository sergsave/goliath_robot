/*
 * hexa_leg.h
 *
 *  Created on: 06 янв. 2018 г.
 *      Author: sergey
 */

#ifndef ROBO_LEG_H_
#define ROBO_LEG_H_

#include "urdf/model.h"
#include <string>
#include <array>

//this class contain a description of insect-like robot's Leg
class RoboLeg
{
public:
  RoboLeg() {}
  RoboLeg(const std::string& leg_prefix, const urdf::Model&);

  struct Position
  {
    Position(double x, double y, double z) : x(x), y(y), z(z) {}
    double x;
    double y;
    double z;
  };
  enum IKResult
  {
    OK = 0,
    WARNING,
    ERROR
  };

  //coxa - latin's name of shoulder, femur - part from shouler to elbow,
  //tibia - forearms analog in insect body
  enum SegmentType
  {
    COXA = 0,
    FEMUR,
    TIBIA,
    NUMBER_OF_SEGMENTS
  };

  //types and methods for return states of all leg's joints
  typedef std::array<double, NUMBER_OF_SEGMENTS> Angles;
  typedef std::array<std::string, NUMBER_OF_SEGMENTS> JntNames;

  //IK - inverse kinematics. Determining a joint's angles from
  //position of leg end
  IKResult getAnglesIK(const Position&, Angles&);
  JntNames getJntNames();

private:
  //this array contain more members than segments in the leg,
  //because there is a additional fixed node - "leg end"
  static const std::array<std::string, NUMBER_OF_SEGMENTS + 1> JNT_BASE_NAMES;

  //indivisible part of leg
  struct Segment
  {
    Segment() : length(0), init_angle(0), min_angle(0), max_angle(0) {}
    double length;
    double init_angle;
    double min_angle;
    double max_angle;
    std::string name;
    bool checkAngle(double angle) const
    {
      return angle <= max_angle && angle >= min_angle;
    }
  };
  std::array<Segment, NUMBER_OF_SEGMENTS> segs_;
  bool checkAngles(Angles&) const;
};

#endif /* ROBO_LEG_H_ */
