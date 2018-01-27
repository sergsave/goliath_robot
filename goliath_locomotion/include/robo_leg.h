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
  enum SegmentType
  {
    COXA = 0,
    FEMUR,
    TIBIA,
    NUMBER_OF_SEGMENTS
  };

  typedef std::array<double, NUMBER_OF_SEGMENTS> Angles;
  typedef std::array<std::string, NUMBER_OF_SEGMENTS> JntNames;

  IKResult getAnglesIK(const Position&, Angles&);
  JntNames getJntNames();

private:
  static const std::array<std::string, NUMBER_OF_SEGMENTS + 1> JNT_BASE_NAMES;

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
