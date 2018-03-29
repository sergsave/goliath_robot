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

  void setGaitType(GaitType gt)
  {
    curr_legs_pos_ = default_legs_pos_;
    curr_phase_step_ = curr_phase_ = 0;

    if (gt == TRIPOD)
      setSequenceToTripod();
    if (gt == RIPPLE)
      setSequenceToRipple();
    if (gt == WAVE)
      setSequenceToWave();
  }

  void accretion(const geometry_msgs::Twist& vel,
                 BodyKinematics::LegsPosition& legs_pos,
                 BodyKinematics::LegsYaw& legs_yaw, double time)
  {
    if (vel.linear.x == 0 && vel.linear.y == 0 && vel.angular.z == 0)
      return;

    LegState temp;

    urdf::Vector3 distance(
        vel.linear.x * time * STEP_NUMBERS * (sequence_.size() - 2),
        vel.linear.y * time * STEP_NUMBERS * (sequence_.size() - 2), 0);

    double yaw = vel.angular.z * time * STEP_NUMBERS * (sequence_.size() - 2);

    for (std::size_t l = BodyKinematics::LF;
         l != BodyKinematics::NUMBER_OF_LEGS; l++)
    {
      urdf::Vector3 back_move_dist(distance.x / (sequence_.size() - 2),
                                   distance.y / (sequence_.size() - 2), 0);
      double back_move_yaw = yaw / (sequence_.size() - 2);

      if (sequence_[curr_phase_][l] == PUT_UP)
      {
        temp = putUpLeg(default_legs_pos_[l], distance, yaw,
                        curr_phase_step_);
      }
      else if (sequence_[curr_phase_][l] == PUT_DOWN)
      {
        temp = putDownLeg(default_legs_pos_[l], distance,yaw,
                          curr_phase_step_);
      }
      else
      {
        temp = returnLeg(curr_legs_pos_[l], curr_legs_yaw_[l],
                         back_move_dist, back_move_yaw,
                         curr_phase_step_);
      }
      legs_pos[l] = temp.lp;
      legs_yaw[l] = temp.yaw;
    }

    /*ROS_ERROR_STREAM(
        "ph " << curr_phase_ << " st " << curr_phase_step_ << " pos.y "
              << legs_pos[0].y - default_legs_pos_[0].y << " pos.z "
              << legs_pos[0].z - default_legs_pos_[0].z);
    */
    if (curr_phase_step_++ == STEP_NUMBERS - 1)
    {
      curr_phase_step_ = 0;
      curr_legs_pos_ = legs_pos;
      curr_legs_yaw_ = legs_yaw;

      if (curr_phase_++ == sequence_.size() - 1)
        curr_phase_ = 0;
    }
  }

private:
  struct LegState
  {
    LegKinematics::LegPos lp;
    double yaw;
  };

  LegState putUpLeg(LegKinematics::LegPos def_pos, urdf::Vector3 dist,
                    double yaw, std::size_t step)
  {
    LegState new_state;

    new_state.lp.x =
        def_pos.x - dist.x / 2 * (STEP_NUMBERS - step) / STEP_NUMBERS;
    new_state.lp.y =
        def_pos.y - dist.y / 2 * (STEP_NUMBERS - step) / STEP_NUMBERS;
    new_state.lp.z = def_pos.z + LIFT_HEIGHT * step / STEP_NUMBERS;
    /*std::size_t mid_step = STEP_NUMBERS / 2;

    if (step < mid_step)
    {
      new_pos.x = def_pos.x - dist.x / 2 * (mid_step - step) / mid_step;
      new_pos.z = def_pos.z + LIFT_HEIGHT * step / mid_step;
    }
    else
    {
      new_pos.x = def_pos.x + dist.x / 2 * (step - mid_step) / mid_step;
      new_pos.z = def_pos.z + LIFT_HEIGHT * (2 * mid_step - step) / mid_step;
    }*/
    new_state.yaw = -yaw / 2 * (STEP_NUMBERS - step) / STEP_NUMBERS;

    return new_state;
  }

  LegState putDownLeg(LegKinematics::LegPos def_pos, urdf::Vector3 dist,
                      double yaw, std::size_t step)
  {
    LegState new_state;

    new_state.lp.x = def_pos.x + dist.x / 2 * (step + 1) / STEP_NUMBERS;
    new_state.lp.y = def_pos.y + dist.y / 2 * (step + 1) / STEP_NUMBERS;
    new_state.lp.z =
        def_pos.z + LIFT_HEIGHT * (STEP_NUMBERS - (step + 1)) / STEP_NUMBERS;

    /*std::size_t mid_step = STEP_NUMBERS / 2;

    if (step < mid_step)
    {
      new_pos.x = def_pos.x - dist.x / 2 * (mid_step - step) / mid_step;
      new_pos.z = def_pos.z + LIFT_HEIGHT * step / mid_step;
    }
    else
    {
      new_pos.x = def_pos.x + dist.x / 2 * (step - mid_step) / mid_step;
      new_pos.z = def_pos.z + LIFT_HEIGHT * (2 * mid_step - step) / mid_step;
    }*/
    new_state.yaw = yaw / 2 * (step + 1) / STEP_NUMBERS;
    return new_state;
  }

  LegState returnLeg(LegKinematics::LegPos pos, double prev_yaw,
                     urdf::Vector3 dist,
                     double yaw, std::size_t step)
  {
    LegState new_state;

    new_state.lp.x = pos.x - dist.x * (step + 1) / STEP_NUMBERS;
    new_state.lp.y = pos.y - dist.y * (step + 1) / STEP_NUMBERS;
    new_state.lp.z = default_legs_pos_[0].z;

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
    new_state.yaw = prev_yaw - yaw * (step + 1) / STEP_NUMBERS;
    return new_state;
  }

  void setSequenceToTripod()
  {
    sequence_.clear();
    Phase phase;
    phase[BodyKinematics::LF] = PUT_UP;
    phase[BodyKinematics::LM] = RETURN;
    phase[BodyKinematics::LR] = PUT_UP;
    phase[BodyKinematics::RF] = RETURN;
    phase[BodyKinematics::RM] = PUT_UP;
    phase[BodyKinematics::RR] = RETURN;
    sequence_.push_back(phase);
    phase[BodyKinematics::LF] = PUT_DOWN;
    phase[BodyKinematics::LM] = RETURN;
    phase[BodyKinematics::LR] = PUT_DOWN;
    phase[BodyKinematics::RF] = RETURN;
    phase[BodyKinematics::RM] = PUT_DOWN;
    phase[BodyKinematics::RR] = RETURN;
    sequence_.push_back(phase);
    phase[BodyKinematics::LF] = RETURN;
    phase[BodyKinematics::LM] = PUT_UP;
    phase[BodyKinematics::LR] = RETURN;
    phase[BodyKinematics::RF] = PUT_UP;
    phase[BodyKinematics::RM] = RETURN;
    phase[BodyKinematics::RR] = PUT_UP;
    sequence_.push_back(phase);
    phase[BodyKinematics::LF] = RETURN;
    phase[BodyKinematics::LM] = PUT_DOWN;
    phase[BodyKinematics::LR] = RETURN;
    phase[BodyKinematics::RF] = PUT_DOWN;
    phase[BodyKinematics::RM] = RETURN;
    phase[BodyKinematics::RR] = PUT_DOWN;
    sequence_.push_back(phase);
  }

  void setSequenceToWave()
  {
    sequence_.clear();
    Phase phase;
    phase[BodyKinematics::LF] = PUT_UP;
    phase[BodyKinematics::LM] = RETURN;
    phase[BodyKinematics::LR] = RETURN;
    phase[BodyKinematics::RF] = RETURN;
    phase[BodyKinematics::RM] = RETURN;
    phase[BodyKinematics::RR] = RETURN;
    sequence_.push_back(phase);
    phase[BodyKinematics::LF] = PUT_DOWN;
    phase[BodyKinematics::LM] = RETURN;
    phase[BodyKinematics::LR] = RETURN;
    phase[BodyKinematics::RF] = RETURN;
    phase[BodyKinematics::RM] = RETURN;
    phase[BodyKinematics::RR] = RETURN;
    sequence_.push_back(phase);

    phase[BodyKinematics::LF] = RETURN;
    phase[BodyKinematics::LM] = PUT_UP;
    phase[BodyKinematics::LR] = RETURN;
    phase[BodyKinematics::RF] = RETURN;
    phase[BodyKinematics::RM] = RETURN;
    phase[BodyKinematics::RR] = RETURN;
    sequence_.push_back(phase);
    phase[BodyKinematics::LF] = RETURN;
    phase[BodyKinematics::LM] = PUT_DOWN;
    phase[BodyKinematics::LR] = RETURN;
    phase[BodyKinematics::RF] = RETURN;
    phase[BodyKinematics::RM] = RETURN;
    phase[BodyKinematics::RR] = RETURN;
    sequence_.push_back(phase);

    phase[BodyKinematics::LF] = RETURN;
    phase[BodyKinematics::LM] = RETURN;
    phase[BodyKinematics::LR] = PUT_UP;
    phase[BodyKinematics::RF] = RETURN;
    phase[BodyKinematics::RM] = RETURN;
    phase[BodyKinematics::RR] = RETURN;
    sequence_.push_back(phase);
    phase[BodyKinematics::LF] = RETURN;
    phase[BodyKinematics::LM] = RETURN;
    phase[BodyKinematics::LR] = PUT_DOWN;
    phase[BodyKinematics::RF] = RETURN;
    phase[BodyKinematics::RM] = RETURN;
    phase[BodyKinematics::RR] = RETURN;
    sequence_.push_back(phase);

    phase[BodyKinematics::LF] = RETURN;
    phase[BodyKinematics::LM] = RETURN;
    phase[BodyKinematics::LR] = RETURN;
    phase[BodyKinematics::RF] = PUT_UP;
    phase[BodyKinematics::RM] = RETURN;
    phase[BodyKinematics::RR] = RETURN;
    sequence_.push_back(phase);
    phase[BodyKinematics::LF] = RETURN;
    phase[BodyKinematics::LM] = RETURN;
    phase[BodyKinematics::LR] = RETURN;
    phase[BodyKinematics::RF] = PUT_DOWN;
    phase[BodyKinematics::RM] = RETURN;
    phase[BodyKinematics::RR] = RETURN;
    sequence_.push_back(phase);

    phase[BodyKinematics::LF] = RETURN;
    phase[BodyKinematics::LM] = RETURN;
    phase[BodyKinematics::LR] = RETURN;
    phase[BodyKinematics::RF] = RETURN;
    phase[BodyKinematics::RM] = PUT_UP;
    phase[BodyKinematics::RR] = RETURN;
    sequence_.push_back(phase);
    phase[BodyKinematics::LF] = RETURN;
    phase[BodyKinematics::LM] = RETURN;
    phase[BodyKinematics::LR] = RETURN;
    phase[BodyKinematics::RF] = RETURN;
    phase[BodyKinematics::RM] = PUT_DOWN;
    phase[BodyKinematics::RR] = RETURN;
    sequence_.push_back(phase);

    phase[BodyKinematics::LF] = RETURN;
    phase[BodyKinematics::LM] = RETURN;
    phase[BodyKinematics::LR] = RETURN;
    phase[BodyKinematics::RF] = RETURN;
    phase[BodyKinematics::RM] = RETURN;
    phase[BodyKinematics::RR] = PUT_UP;
    sequence_.push_back(phase);
    phase[BodyKinematics::LF] = RETURN;
    phase[BodyKinematics::LM] = RETURN;
    phase[BodyKinematics::LR] = RETURN;
    phase[BodyKinematics::RF] = RETURN;
    phase[BodyKinematics::RM] = RETURN;
    phase[BodyKinematics::RR] = PUT_DOWN;
    sequence_.push_back(phase);
  }

  void setSequenceToRipple()
  {
    sequence_.clear();
    Phase phase;
    phase[BodyKinematics::LF] = RETURN;
    phase[BodyKinematics::LM] = PUT_DOWN;
    phase[BodyKinematics::LR] = RETURN;
    phase[BodyKinematics::RF] = RETURN;
    phase[BodyKinematics::RM] = RETURN;
    phase[BodyKinematics::RR] = PUT_UP;
    sequence_.push_back(phase);
    phase[BodyKinematics::LF] = PUT_UP;
    phase[BodyKinematics::LM] = RETURN;
    phase[BodyKinematics::LR] = RETURN;
    phase[BodyKinematics::RF] = RETURN;
    phase[BodyKinematics::RM] = RETURN;
    phase[BodyKinematics::RR] = PUT_DOWN;
    sequence_.push_back(phase);
    phase[BodyKinematics::LF] = PUT_DOWN;
    phase[BodyKinematics::LM] = RETURN;
    phase[BodyKinematics::LR] = RETURN;
    phase[BodyKinematics::RF] = RETURN;
    phase[BodyKinematics::RM] = PUT_UP;
    phase[BodyKinematics::RR] = RETURN;
    sequence_.push_back(phase);
    phase[BodyKinematics::LF] = RETURN;
    phase[BodyKinematics::LM] = RETURN;
    phase[BodyKinematics::LR] = PUT_UP;
    phase[BodyKinematics::RF] = RETURN;
    phase[BodyKinematics::RM] = PUT_DOWN;
    phase[BodyKinematics::RR] = RETURN;
    sequence_.push_back(phase);
    phase[BodyKinematics::LF] = RETURN;
    phase[BodyKinematics::LM] = RETURN;
    phase[BodyKinematics::LR] = PUT_DOWN;
    phase[BodyKinematics::RF] = PUT_UP;
    phase[BodyKinematics::RM] = RETURN;
    phase[BodyKinematics::RR] = RETURN;
    sequence_.push_back(phase);
    phase[BodyKinematics::LF] = RETURN;
    phase[BodyKinematics::LM] = PUT_UP;
    phase[BodyKinematics::LR] = RETURN;
    phase[BodyKinematics::RF] = PUT_DOWN;
    phase[BodyKinematics::RM] = RETURN;
    phase[BodyKinematics::RR] = RETURN;
    sequence_.push_back(phase);
  }

  enum STROKE
  {
    PUT_UP,
    PUT_DOWN,
    RETURN
  };
  typedef std::array<STROKE, BodyKinematics::NUMBER_OF_LEGS> Phase;

  static const int STEP_NUMBERS = 3;
  static const double LIFT_HEIGHT;
  std::size_t curr_phase_step_;
  std::size_t curr_phase_;
  std::vector<Phase> sequence_;
  BodyKinematics::LegsPosition default_legs_pos_;
  BodyKinematics::LegsPosition curr_legs_pos_;
  BodyKinematics::LegsYaw curr_legs_yaw_;
};

#endif // GAIT_GENERATOR_H
