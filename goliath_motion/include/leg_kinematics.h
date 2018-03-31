/*
 * hexa_leg.h
 *
 *  Created on: 06 янв. 2018 г.
 *      Author: sergey
 */

#ifndef LEG_KINEMATICS_H_
#define LEG_KINEMATICS_H_

#include "urdf/model.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
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

  typedef urdf::Vector3 LegPos;

  // IK - inverse kinematics. Determining a joint's angles from
  // position of leg end. It pushes 3 angles (coxa, femur, tibia)
  // in leg_angles.positions[] vector.
  IKResult
  calculateJntAngles(const LegPos& leg_pos,
                     trajectory_msgs::JointTrajectoryPoint& leg_angles);

  LegPos getDefaultPos() { return def_pos_; }
  void getJntNames(std::vector<std::string>& names);

private:
  // this array contain more members than segments in the leg,
  // because there is a additional fixed node - "leg end"
  static const std::array<std::string, NUMBER_OF_SEGMENTS + 1> JNT_BASE_NAMES;

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

  // forward kinematics
  LegPos calculateDefaultPos();
  // crash protection
  bool checkAngles(Angles&) const;

  std::array<Segment, NUMBER_OF_SEGMENTS> segs_;
  LegPos def_pos_;
};

#endif /* LEG_KINEMATICS_H_ */
