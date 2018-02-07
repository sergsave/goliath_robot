/*
 * hexa_leg.h
 *
 *  Created on: 06 янв. 2018 г.
 *      Author: sergey
 */

#ifndef LEG_KINEMATICS_H_
#define LEG_KINEMATICS_H_

#include "urdf/model.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/JointState.h"
#include <string>
#include <array>

// this class contain a description of insect-like robot's Leg
class LegKinematics
{
public:
  LegKinematics() {}
  LegKinematics(const std::string& leg_prefix, const urdf::Model&);

  enum IKResult
  {
    OK = 0,
    WARNING,
    ERROR
  };

  // IK - inverse kinematics. Determining a joint's angles from
  // position of leg end
  IKResult calculateJntAngles(const geometry_msgs::Point32& leg_pos,
                              sensor_msgs::JointState& leg_angles);

  // forward kinematics
  geometry_msgs::Point32 calculateDefaultPos();

private:
  // coxa - latin's name of shoulder, femur - part from shouler to elbow,
  // tibia - forearms analog in insect body
  enum SegmentType
  {
    COXA = 0,
    FEMUR,
    TIBIA,
    NUMBER_OF_SEGMENTS
  };

  // indivisible part of leg
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

  typedef std::array<double, NUMBER_OF_SEGMENTS> Angles;

  // this array contain more members than segments in the leg,
  // because there is a additional fixed node - "leg end"
  static const std::array<std::string, NUMBER_OF_SEGMENTS + 1> JNT_BASE_NAMES;
  std::array<Segment, NUMBER_OF_SEGMENTS> segs_;

  bool checkAngles(Angles&) const;
};

#endif /* LEG_KINEMATICS_H_ */