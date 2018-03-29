#include "gait_generator.h"

using std::size_t;
const double GaitGenerator::LIFT_HEIGHT = 0.02;

void GaitGenerator::accretion(const geometry_msgs::Twist& vel,
                              BodyKinematics::LegsStance& legs_st, double dt)
{
  if (vel.linear.x == 0 && vel.linear.y == 0 && vel.angular.z == 0)
    return;

  double time_for_phase = dt * STEP_NUMBERS;
  double time_for_cycle = time_for_phase * (sequence_.size() - 2);

  urdf::Pose cycle_delta = calcDelta(vel, time_for_cycle);
  urdf::Pose phase_delta = calcDelta(vel, time_for_phase);

  for (size_t l = BodyKinematics::LF; l != BodyKinematics::NUMBER_OF_LEGS; l++)
  {
    if (sequence_[curr_phase_][l] == PUT_UP)
    {
      legs_st[l] = putUpLeg(default_legs_st_[l], cycle_delta, curr_phase_step_);
    }
    else if (sequence_[curr_phase_][l] == PUT_DO)
    {
      legs_st[l] =
          putDownLeg(default_legs_st_[l], cycle_delta, curr_phase_step_);
    }
    else
    {
      legs_st[l] = returnLeg(curr_legs_st_[l], phase_delta, curr_phase_step_);
    }
  }

  if (curr_phase_step_++ == STEP_NUMBERS - 1)
  {
    curr_phase_step_ = 0;
    curr_legs_st_ = legs_st;

    if (curr_phase_++ == sequence_.size() - 1)
      curr_phase_ = 0;
  }
}

void GaitGenerator::setType(GaitType gt)
{
  curr_legs_st_ = default_legs_st_;
  curr_phase_step_ = curr_phase_ = 0;

  if (gt == TRIPOD)
    setSequenceToTripod();
  if (gt == RIPPLE)
    setSequenceToRipple();
  if (gt == WAVE)
    setSequenceToWave();
}

void GaitGenerator::setSequenceToTripod()
{
  sequence_.clear();
  Phase phase;
  // based on https://insectsandrobots.weebly.com/tripod-gait.html
  // legs:   LF      LM      LR      RF      RM      RR
  phase = {PUT_UP, RETURN, PUT_UP, RETURN, PUT_UP, RETURN};
  sequence_.push_back(phase);
  phase = {PUT_DO, RETURN, PUT_DO, RETURN, PUT_DO, RETURN};
  sequence_.push_back(phase);
  phase = {RETURN, PUT_UP, RETURN, PUT_UP, RETURN, PUT_UP};
  sequence_.push_back(phase);
  phase = {RETURN, PUT_DO, RETURN, PUT_DO, RETURN, PUT_DO};
  sequence_.push_back(phase);
}

void GaitGenerator::setSequenceToWave()
{
  sequence_.clear();
  Phase phase;
  // based on https://insectsandrobots.weebly.com/tripod-gait.html
  // legs:   LF      LM      LR      RF      RM      RR
  phase = {PUT_UP, RETURN, RETURN, RETURN, RETURN, RETURN};
  sequence_.push_back(phase);
  phase = {PUT_DO, RETURN, RETURN, RETURN, RETURN, RETURN};
  sequence_.push_back(phase);
  phase = {RETURN, PUT_UP, RETURN, RETURN, RETURN, RETURN};
  sequence_.push_back(phase);
  phase = {RETURN, PUT_DO, RETURN, RETURN, RETURN, RETURN};
  sequence_.push_back(phase);
  phase = {RETURN, RETURN, PUT_UP, RETURN, RETURN, RETURN};
  sequence_.push_back(phase);
  phase = {RETURN, RETURN, PUT_DO, RETURN, RETURN, RETURN};
  sequence_.push_back(phase);
  phase = {RETURN, RETURN, RETURN, PUT_UP, RETURN, RETURN};
  sequence_.push_back(phase);
  phase = {RETURN, RETURN, RETURN, PUT_DO, RETURN, RETURN};
  sequence_.push_back(phase);
  phase = {RETURN, RETURN, RETURN, RETURN, PUT_UP, RETURN};
  sequence_.push_back(phase);
  phase = {RETURN, RETURN, RETURN, RETURN, PUT_DO, RETURN};
  sequence_.push_back(phase);
  phase = {RETURN, RETURN, RETURN, RETURN, RETURN, PUT_UP};
  sequence_.push_back(phase);
  phase = {RETURN, RETURN, RETURN, RETURN, RETURN, PUT_DO};
  sequence_.push_back(phase);
}

void GaitGenerator::setSequenceToRipple()
{
  sequence_.clear();
  Phase phase;
  // based on https://insectsandrobots.weebly.com/tripod-gait.html
  // legs:   LF      LM      LR      RF      RM      RR
  phase = {RETURN, PUT_DO, RETURN, RETURN, RETURN, PUT_UP};
  sequence_.push_back(phase);
  phase = {PUT_UP, RETURN, RETURN, RETURN, RETURN, PUT_DO};
  sequence_.push_back(phase);
  phase = {PUT_DO, RETURN, RETURN, RETURN, PUT_UP, RETURN};
  sequence_.push_back(phase);
  phase = {RETURN, RETURN, PUT_UP, RETURN, PUT_DO, RETURN};
  sequence_.push_back(phase);
  phase = {RETURN, RETURN, PUT_DO, PUT_UP, RETURN, RETURN};
  sequence_.push_back(phase);
  phase = {RETURN, PUT_UP, RETURN, PUT_DO, RETURN, RETURN};
  sequence_.push_back(phase);
}

double GaitGenerator::getYaw(const urdf::Pose& pose)
{
  double roll, pitch, yaw;
  pose.rotation.getRPY(roll, pitch, yaw);
  return yaw;
}

urdf::Pose GaitGenerator::calcDelta(const geometry_msgs::Twist& vel, double dt)
{
  urdf::Pose delta;
  delta.position = urdf::Vector3(vel.linear.x * dt, vel.linear.y * dt, 0);
  delta.rotation.setFromRPY(0, 0, vel.angular.z * dt);
  return delta;
}

urdf::Pose GaitGenerator::getHalfDelta(const urdf::Pose& delta)
{
  urdf::Pose ret = delta;
  ret.position.x /= 2;
  ret.position.y /= 2;
  ret.rotation.setFromRPY(0, 0, getYaw(delta) / 2);
  return ret;
}

void GaitGenerator::moveLeg(BodyKinematics::StanceOfLeg& state,
                            urdf::Pose delta, double move_koef,
                            double lift_koef)
{
  state.pos.x += move_koef * delta.position.x;
  state.pos.y += move_koef * delta.position.y;
  state.rot_offset += move_koef * getYaw(delta);

  state.pos.z += LIFT_HEIGHT * lift_koef;
}

BodyKinematics::StanceOfLeg
GaitGenerator::putUpLeg(BodyKinematics::StanceOfLeg def_st, urdf::Pose delta,
                        std::size_t step)
{
  BodyKinematics::StanceOfLeg new_st = def_st;
  double move_koef = -(double)(STEP_NUMBERS - step) / STEP_NUMBERS;
  double lift_koef = (double)step / STEP_NUMBERS;

  moveLeg(new_st, getHalfDelta(delta), move_koef, lift_koef);
  return new_st;
}

BodyKinematics::StanceOfLeg
GaitGenerator::putDownLeg(BodyKinematics::StanceOfLeg def_st, urdf::Pose delta,
                          std::size_t step)
{
  BodyKinematics::StanceOfLeg new_st = def_st;
  double move_koef = (double)(step + 1) / STEP_NUMBERS;
  double lift_koef = (double)(STEP_NUMBERS - (step + 1)) / STEP_NUMBERS;

  moveLeg(new_st, getHalfDelta(delta), move_koef, lift_koef);
  return new_st;
}

BodyKinematics::StanceOfLeg
GaitGenerator::returnLeg(BodyKinematics::StanceOfLeg prev_st, urdf::Pose delta,
                         std::size_t step)
{
  BodyKinematics::StanceOfLeg new_st = prev_st;
  double move_koef = -(double)(step + 1) / STEP_NUMBERS;

  moveLeg(new_st, delta, move_koef, 0);
  new_st.pos.z = default_legs_st_[0].pos.z;
  return new_st;
}
