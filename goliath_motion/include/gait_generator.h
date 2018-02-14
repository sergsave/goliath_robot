#ifndef GAIT_GENERATOR_H
#define GAIT_GENERATOR_H

#include "geometry_msgs/Twist.h"
#include "goliath_msgs/LegsPosition.h"
#include <body_kinematics.h>

class GaitGenerator
{
public:
  GaitGenerator(BodyKinematics& body):
  default_legs_pos_(body.getDefaultLegsPos())
  {

  }

  enum GaitType
  {
    TRIPOD,
    WAVE,
    RIPPLE
  };

  void setGaitType(GaitType gt);
  void iteration(const geometry_msgs::Twist&,
                 const goliath_msgs::LegsPosition&);

private:
  goliath_msgs::LegsPosition default_legs_pos_;
};

#endif // GAIT_GENERATOR_H
