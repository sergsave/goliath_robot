#ifndef GAIT_GENERATOR_H
#define GAIT_GENERATOR_H

#include "geometry_msgs/Twist.h"
#include <body_kinematics.h>

class GaitGenerator
{
public:
  GaitGenerator() {}

  GaitGenerator(BodyKinematics& body)
      : default_legs_pos_(body.getDefaultLegsPos()),
        curr_legs_pos_(body.getDefaultLegsPos()), curr_phase_step_(0),
        curr_phase_(0)
  {
    setSequenceToTripod();
  }

  enum GaitType
  {
    TRIPOD,
    WAVE,
    RIPPLE
  };

  void setGaitType(GaitType gt);

  void accretion(const geometry_msgs::Twist& vel,
                 BodyKinematics::LegsPosition& legs_pos, double time)
  {
    urdf::Vector3 distance(
        vel.linear.x * time * (STEP_NUMBERS - 1) * (sequence_.size() - 1),
        vel.linear.y * time * (STEP_NUMBERS - 1) * (sequence_.size() - 1), 0);

    if (distance.x == 0)
      return;

    for (std::size_t l = BodyKinematics::LF;
         l != BodyKinematics::NUMBER_OF_LEGS; l++)
    {
      urdf::Vector3 back_move_dist(distance.x / (sequence_.size() - 1),
                                   distance.y / (sequence_.size() - 1), 0);

      if (sequence_[curr_phase_][l])
      {
        legs_pos[l] = raiseAndPutDownLeg(default_legs_pos_[l], distance,
                                         curr_phase_step_);
      }
      else
      {
        legs_pos[l] =
            moveBackLeg(curr_legs_pos_[l], back_move_dist, curr_phase_step_);
      }
    }
    ROS_ERROR_STREAM("ph" << curr_phase_ << "st" << curr_phase_step_ << "pos.x"
                          << legs_pos[0].x << "pos.z" << legs_pos[0].z);

    if (curr_phase_step_++ == STEP_NUMBERS - 1)
    {
      curr_phase_step_ = 0;
      curr_legs_pos_ = legs_pos;

      if (curr_phase_++ == sequence_.size() - 1)
        curr_phase_ = 0;
    }
  }

private:
  LegKinematics::LegPos raiseAndPutDownLeg(LegKinematics::LegPos def_pos,
                                           urdf::Vector3 dist, std::size_t step)
  {
    LegKinematics::LegPos new_pos;
    std::size_t mid_step = STEP_NUMBERS / 2;

    if (step < mid_step)
    {
      new_pos.x = def_pos.x - dist.x / 2 * (mid_step - step) / mid_step;
      new_pos.z = def_pos.z + LIFT_HEIGHT * step / mid_step;
    }
    else
    {
      new_pos.x = def_pos.x + dist.x / 2 * (step - mid_step) / mid_step;
      new_pos.z = def_pos.z + LIFT_HEIGHT * (2 * mid_step - step) / mid_step;
    }

    new_pos.y = def_pos.y;
    return new_pos;
  }

  LegKinematics::LegPos moveBackLeg(LegKinematics::LegPos pos,
                                    urdf::Vector3 dist, std::size_t step)
  {
    LegKinematics::LegPos new_pos;

    new_pos.x = pos.x - dist.x * step / (STEP_NUMBERS - 1);
    new_pos.z = default_legs_pos_[0].z;
    new_pos.y = pos.y;

    /*std::size_t mid_step = STEP_NUMBERS / 2;

    if (step < mid_step)
    {
      new_pos.x = pos.x + dist.x / 2 * (mid_step - step) / mid_step;
      new_pos.z = pos.z;
    }
    else
    {
      new_pos.x = pos.x - dist.x / 2 * (step - mid_step) / mid_step;
      new_pos.z = pos.z;
    }*/

    return new_pos;
  }

  void setSequenceToTripod()
  {
    sequence_.clear();
    Phase phase;
    phase[BodyKinematics::LF] = true;
    phase[BodyKinematics::LM] = false;
    phase[BodyKinematics::LR] = true;
    phase[BodyKinematics::RF] = false;
    phase[BodyKinematics::RM] = true;
    phase[BodyKinematics::RR] = false;
    sequence_.push_back(phase);
    phase[BodyKinematics::LF] = false;
    phase[BodyKinematics::LM] = true;
    phase[BodyKinematics::LR] = false;
    phase[BodyKinematics::RF] = true;
    phase[BodyKinematics::RM] = false;
    phase[BodyKinematics::RR] = true;
    sequence_.push_back(phase);
  }

  void setSequenceToWave()
  {
    sequence_.clear();
    Phase phase;
    phase[BodyKinematics::LF] = true;
    phase[BodyKinematics::LM] = false;
    phase[BodyKinematics::LR] = false;
    phase[BodyKinematics::RF] = false;
    phase[BodyKinematics::RM] = false;
    phase[BodyKinematics::RR] = false;
    sequence_.push_back(phase);
    phase[BodyKinematics::LF] = false;
    phase[BodyKinematics::LM] = true;
    phase[BodyKinematics::LR] = false;
    phase[BodyKinematics::RF] = false;
    phase[BodyKinematics::RM] = false;
    phase[BodyKinematics::RR] = false;
    sequence_.push_back(phase);
    phase[BodyKinematics::LF] = false;
    phase[BodyKinematics::LM] = false;
    phase[BodyKinematics::LR] = true;
    phase[BodyKinematics::RF] = false;
    phase[BodyKinematics::RM] = false;
    phase[BodyKinematics::RR] = false;
    sequence_.push_back(phase);
    phase[BodyKinematics::LF] = false;
    phase[BodyKinematics::LM] = false;
    phase[BodyKinematics::LR] = false;
    phase[BodyKinematics::RF] = true;
    phase[BodyKinematics::RM] = false;
    phase[BodyKinematics::RR] = false;
    sequence_.push_back(phase);
    phase[BodyKinematics::LF] = false;
    phase[BodyKinematics::LM] = false;
    phase[BodyKinematics::LR] = false;
    phase[BodyKinematics::RF] = false;
    phase[BodyKinematics::RM] = true;
    phase[BodyKinematics::RR] = false;
    sequence_.push_back(phase);
    phase[BodyKinematics::LF] = false;
    phase[BodyKinematics::LM] = false;
    phase[BodyKinematics::LR] = false;
    phase[BodyKinematics::RF] = false;
    phase[BodyKinematics::RM] = false;
    phase[BodyKinematics::RR] = true;
    sequence_.push_back(phase);
  }
  // must be odd!
  static const int STEP_NUMBERS = 5;

  static const double LIFT_HEIGHT;
  std::size_t curr_phase_step_;
  std::size_t curr_phase_;

  typedef std::array<bool, BodyKinematics::NUMBER_OF_LEGS> Phase;

  std::vector<Phase> sequence_;

  BodyKinematics::LegsPosition default_legs_pos_;
  BodyKinematics::LegsPosition curr_legs_pos_;
};

#endif // GAIT_GENERATOR_H
