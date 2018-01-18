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

class HexaLeg
{
public:
  HexaLeg(const std::string& leg_prefix, const urdf::Model&);

  struct Angles
  {
    Angles() : coxa_(0), femur_(0), tibia_(0) {}
    double coxa_;
    double femur_;
    double tibia_;
  };
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

  IKResult getAnglesIK(const Position&, Angles&); // calculate inverse
                                                  // kinematics
  struct JntNames
  {
    std::string coxa_;
    std::string femur_;
    std::string tibia_;
  };
  JntNames getJntNames();

  private :
  static const std::string COXA_J_NAME;
  static const std::string FEMUR_J_NAME;
  static const std::string TIBIA_J_NAME;
  static const std::string TARSUS_J_NAME;

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
  Segment coxa_, femur_, tibia_;
  bool checkAngles(Angles& angs);
};

#endif /* HEXA_LEG_H_ */
