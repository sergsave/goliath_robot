/*
 * hexa_leg.h
 *
 *  Created on: 06 янв. 2018 г.
 *      Author: sergey
 */

#ifndef HEXA_LEG_H_
#define HEXA_LEG_H_

#include "urdf/model.h"
#include <string>
#include <array>

class HexaLeg
{
public:
  HexaLeg(const std::string& leg_prefix, const urdf::Model&);

  struct Position
  {
    Position(float x, float y, float z) : x_(x), y_(y), z_(z) {}
    double x_;
    double y_;
    double z_;
  };
  enum IKResult
  {
    OK = 0,
    Warning,
    Error
  };
  enum SegmentType
  {
    Coxa = 0,
    Femur,
    Tibia,
    LegEnd
  };

  static const int NUMBER_OF_SEGMENTS = LegEnd;
  typedef std::array<double, NUMBER_OF_SEGMENTS> Angles;
  typedef std::array<std::string, NUMBER_OF_SEGMENTS> JntNames;

  IKResult getAnglesIK(const Position&, Angles&);
  JntNames getJntNames();
private:

  static const std::array<std::string, NUMBER_OF_SEGMENTS + 1> JNT_BASE_NAMES;

  struct Segment
  {
    Segment() : length_(0), init_angle_(0), min_angle_(0), max_angle_(0){};
    double length_;
    double init_angle_;
    double min_angle_;
    double max_angle_;
    std::string name;
    bool checkAngle(double angle)
    {
      return angle <= max_angle_ && angle >= min_angle_;
    };
  };
  std::array<Segment, NUMBER_OF_SEGMENTS> segs_;
  bool checkAngles(Angles&);
};

#endif /* HEXA_LEG_H_ */
