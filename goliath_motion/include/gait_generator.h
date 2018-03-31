#ifndef GAIT_GENERATOR_H
#define GAIT_GENERATOR_H

#include "geometry_msgs/Twist.h"
#include <body_kinematics.h>

// The class is responsible for the walk of robots.
// There are 3 types of gait:
// Tripod is the fastest, but eats a lot of energy.
// Wave is the slowest, but energy-efficient.
// Ripple is the optimal variant.
// The main function of class - calculate legs position based on
// gait velocity and gait type.
class GaitGenerator
{
public:
  GaitGenerator() {}

  GaitGenerator(BodyKinematics& body)
      : default_legs_st_(body.getDefaultLegsStance()),
        curr_legs_st_(body.getDefaultLegsStance()), curr_phase_step_(0),
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

  void setType(GaitType gt);
  // main method of gait class, just periodically call it for walk
  void accretion(const geometry_msgs::Twist& vel,
                 BodyKinematics::LegsStance& legs_st, double dt);

private:
  enum STROKE
  {
    PUT_UP,
    PUT_DO,
    RETURN
  };
  typedef std::array<STROKE, BodyKinematics::NUMBER_OF_LEGS> Phase;

  void setSequenceToTripod();
  void setSequenceToWave();
  void setSequenceToRipple();

  double getYaw(const urdf::Pose& pose);
  urdf::Pose calcDelta(const geometry_msgs::Twist& vel, double dt);
  urdf::Pose getHalfDelta(const urdf::Pose& delta);

  void moveLeg(BodyKinematics::StanceOfLeg& state, urdf::Pose delta,
               double move_koef, double lift_koef);

  BodyKinematics::StanceOfLeg putUpLeg(BodyKinematics::StanceOfLeg def_st,
                                       urdf::Pose delta, std::size_t step);

  BodyKinematics::StanceOfLeg putDownLeg(BodyKinematics::StanceOfLeg def_st,
                                         urdf::Pose delta, std::size_t step);

  BodyKinematics::StanceOfLeg returnLeg(BodyKinematics::StanceOfLeg prev_st,
                                        urdf::Pose delta, std::size_t step);

  static const int STEP_NUMBERS = 3;
  static const double LIFT_HEIGHT;
  // phase step is smallest part of gait cycle
  // phase it's like put down leg, or put up leg
  std::size_t curr_phase_step_;
  std::size_t curr_phase_;
  // sequence contain gait algorithm
  std::vector<Phase> sequence_;
  BodyKinematics::LegsStance default_legs_st_;
  BodyKinematics::LegsStance curr_legs_st_;
};

#endif // GAIT_GENERATOR_H
