/*
 * goliath_motion.cpp
 *
 *  Created on: 06 янв. 2018 г.
 *      Author: sergey
 */

#include "ros/ros.h"
#include <string>
#include <array>
#include <tf/transform_broadcaster.h>
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/Twist.h"
#include "goliath_msgs/LegsVel.h"
#include "goliath_msgs/MotionCmd.h"
#include "body_kinematics.h"
#include "gait_generator.h"

using std::string;
using std::endl;

// This class contain motion control mechanism
// for Goliath - 6 leg's insect's like robot.
// Publish in topic: command (for motor driver).
// Subscribe on topics: legs_cmd_vel, body_cmd_vel,
// gait_cmd_vel (for motion control) and motion_cmd
// for select gait types.
class GoliathMotion
{
public:
  GoliathMotion(ros::NodeHandle node)
      : nh_(node), private_nh_("~"),
        last_gait_or_legs_command_time_(ros::Time::now()),
        last_body_command_time_(ros::Time::now()), auto_lvl_state_(IDLE)
  {
    urdf::Model model;

    // use parameter from ROS parameter server for create URDF-model.
    // if an xml robot's description is not added to server, and error
    // will occur
    if (!model.initParam("robot_description"))
    {
      ROS_FATAL_STREAM("Need a \"robot description\" parameter on server"
                       << endl);
      exit(-1);
    }
#warning "create this objects dynamicly"
    body_ = BodyKinematics(model);
    gait_ = GaitGenerator(body_);
    curr_legs_stance_ = body_.getDefaultLegsStance();

    // create public's publisher and subscriber
    gait_vel_sub_ = nh_.subscribe("gait_cmd_vel", SUB_QUEUE_SZ,
                                  &GoliathMotion::gaitVelCallback, this);

    body_vel_sub_ = nh_.subscribe("body_cmd_vel", SUB_QUEUE_SZ,
                                  &GoliathMotion::bodyVelCallback, this);

    legs_vel_sub_ = nh_.subscribe("legs_cmd_vel", SUB_QUEUE_SZ,
                                  &GoliathMotion::legsVelCallback, this);

    // for manual set use in command line:
    // rostopic pub -1 /motion_cmd goliath_msgs/MotionCmd -- '2'
    cmd_sub_ = nh_.subscribe("motion_cmd", SUB_QUEUE_SZ,
                             &GoliathMotion::motionCmdCallback, this);

    jnt_traj_pub_ = private_nh_.advertise<trajectory_msgs::JointTrajectory>(
        "command", JNT_TRAJ_QUEUE_SZ);
  }

  // main loop method
  void spin(void)
  {
    while (ros::ok())
    {
      updateAndMove();
      // stop the robot if there are no commands
      resetVelocities();
      ros::spinOnce();
    }
  }

private:
  static const int SUB_QUEUE_SZ = 10;
  static const int JNT_TRAJ_QUEUE_SZ = 10;
  static const int TF_QUANTITY_PER_STEP = 5;
  static const double MOVE_TIME_STEP, VEL_RESET_TIMEOUT;
  static const int AUTO_LEVELING_STEPS_QUANTITY = 10;

  enum BodyAutoLevelingState
  {
    DISABLE,
    IDLE,
    IN_PROCESS
  };

  void updateAndMove()
  {
    BodyKinematics::LegsStance new_legs_stance = curr_legs_stance_;
    BodyKinematics::BodyPose new_body_pose = curr_body_pose_;
    urdf::Pose new_travel_state = curr_travel_state_;

    // shift the limbs of the robot based on speed: s = v * t
    shiftLegsStance(new_legs_stance, MOVE_TIME_STEP);
    shiftBodyPose(new_body_pose, MOVE_TIME_STEP);
    shiftTravelState(new_travel_state, MOVE_TIME_STEP);

    // do a piece of step
    gait_.accretion(gait_velocity_, new_legs_stance, MOVE_TIME_STEP);

    trajectory_msgs::JointTrajectory traj;
    // let's try to create a trajectory for robot's limbs.
    // if all angles is OK, publish this trajectory to motor controller.
    // if there are a bad angles, just wait.
    if (createJntTraj(new_legs_stance, new_body_pose, MOVE_TIME_STEP, traj))
    {
      jnt_traj_pub_.publish(traj);
      // Update a position of a robot in space
      waitAndPublishTf(MOVE_TIME_STEP, curr_body_pose_, curr_travel_state_);

      curr_legs_stance_ = new_legs_stance;
      curr_body_pose_ = new_body_pose;
      curr_travel_state_ = new_travel_state;
    }
    else
      ros::Duration(MOVE_TIME_STEP).sleep();
  }

  // stop the robot if there are no commands
  void resetVelocities()
  {
    if (ros::Time::now() - last_gait_or_legs_command_time_ >
        ros::Duration(VEL_RESET_TIMEOUT))
    {
      // set vel to zero
      legs_velocity_ = goliath_msgs::LegsVel();
      gait_velocity_ = geometry_msgs::Twist();
    }

    if (ros::Time::now() - last_body_command_time_ >
        ros::Duration(VEL_RESET_TIMEOUT))
    {
      // if autoleveling is enable - move body to default pose
      switch (auto_lvl_state_)
      {
      case IDLE:
        if (!isBodyPoseDefault())
        {
          startAutoLeveling();
          auto_lvl_state_ = IN_PROCESS;
        }
        break;

      case IN_PROCESS:
        if (isBodyPoseDefault())
        {
          body_velocity_ = geometry_msgs::Twist();
          auto_lvl_state_ = IDLE;
        }
        break;

      case DISABLE:
        // just reset to zero
        body_velocity_ = geometry_msgs::Twist();
        break;

      default:
        break;
      }
    }
  }

  // move body to default pose
  void startAutoLeveling()
  {
    static const double alv_time =
        AUTO_LEVELING_STEPS_QUANTITY * MOVE_TIME_STEP;

    double r, p, y;
    curr_body_pose_.rotation.getRPY(r, p, y);
    body_velocity_.angular.x = -r / alv_time;
    body_velocity_.angular.y = -p / alv_time;
    body_velocity_.angular.z = -y / alv_time;

    body_velocity_.linear.x = -curr_body_pose_.position.x / alv_time;
    body_velocity_.linear.y = -curr_body_pose_.position.y / alv_time;
    body_velocity_.linear.z = -curr_body_pose_.position.z / alv_time;
  }

  // maybe change it to lambda?
  bool isZero(double d) { return d < 0.0001 && d > -0.0001; }

  bool isBodyPoseDefault()
  {
    double r, p, y;
    curr_body_pose_.rotation.getRPY(r, p, y);
    urdf::Vector3 pos = curr_body_pose_.position;

    return isZero(pos.x) && isZero(pos.y) && isZero(pos.z) && isZero(r) &&
           isZero(p) && isZero(y);
  }

  // all callbacks are automatically called in spin() method
  // with call of SpinOnce()
  void gaitVelCallback(const geometry_msgs::Twist& tw)
  {
    gait_velocity_ = tw;
    last_gait_or_legs_command_time_ = ros::Time::now();
  }

  void bodyVelCallback(const geometry_msgs::Twist& tw)
  {
    body_velocity_ = tw;
    last_body_command_time_ = ros::Time::now();
    if (auto_lvl_state_ != DISABLE)
      auto_lvl_state_ = IDLE;
  }

  void legsVelCallback(const goliath_msgs::LegsVel& legs_vel)
  {
    legs_velocity_ = legs_vel;
    last_gait_or_legs_command_time_ = ros::Time::now();
  }

  void motionCmdCallback(const goliath_msgs::MotionCmd& cmd)
  {
    if (cmd.type == goliath_msgs::MotionCmd::SELECT_TRIPOD_GAIT)
      gait_.setType(GaitGenerator::TRIPOD);
    else if (cmd.type == goliath_msgs::MotionCmd::SELECT_WAVE_GAIT)
      gait_.setType(GaitGenerator::WAVE);
    else if (cmd.type == goliath_msgs::MotionCmd::SELECT_RIPPLE_GAIT)
      gait_.setType(GaitGenerator::RIPPLE);
    else if (cmd.type == goliath_msgs::MotionCmd::SWITCH_BODY_AUTOLEVELING)
      auto_lvl_state_ = (auto_lvl_state_ == DISABLE) ? IDLE : DISABLE;
  }

  // change Legs, Body position according to speed and delta t
  void shiftLegsStance(BodyKinematics::LegsStance& new_ls, double dt)
  {
    for (std::size_t l = 0; l != new_ls.size(); ++l)
    {
      LegKinematics::LegPos delta(dt * legs_velocity_.velocities[l].x,
                                  dt * legs_velocity_.velocities[l].y,
                                  dt * legs_velocity_.velocities[l].z);
      new_ls[l].pos = new_ls[l].pos + delta;
    }
  }

  void shiftBodyPose(BodyKinematics::BodyPose& new_bp, double dt)
  {
    double roll, pitch, yaw;
    new_bp.rotation.getRPY(roll, pitch, yaw);
    new_bp.rotation.setFromRPY(roll + dt * body_velocity_.angular.x,
                               pitch + dt * body_velocity_.angular.y,
                               yaw + dt * body_velocity_.angular.z);

    urdf::Vector3 delta(dt * body_velocity_.linear.x,
                        dt * body_velocity_.linear.y,
                        dt * body_velocity_.linear.z);
    new_bp.position = new_bp.position + delta;
  }

  // change robot's position in a space according to speed and delta t
  void shiftTravelState(urdf::Pose& new_st, double dt)
  {
    double roll, pitch, yaw;
    new_st.rotation.getRPY(roll, pitch, yaw);
    new_st.rotation.setFromRPY(0, 0, yaw + dt * gait_velocity_.angular.z);

    urdf::Vector3 delta(dt * gait_velocity_.linear.x,
                        dt * gait_velocity_.linear.y, 0);

    // rotate velocity vector!
    delta = new_st.rotation * delta;

    new_st.position = new_st.position + delta;
  }

  // try to generate move trajectory
  bool createJntTraj(const BodyKinematics::LegsStance& ls,
                     const BodyKinematics::BodyPose& bp, double dur,
                     trajectory_msgs::JointTrajectory& traj)
  {
    trajectory_msgs::JointTrajectoryPoint traj_point;

    try
    {
      body_.calculateJntAngles(bp, ls, traj_point);
    }
    catch (std::logic_error e)
    {
      ROS_ERROR_STREAM(e.what());
      return false;
    }

    body_.getLegsJntName(traj.joint_names);
    traj.header.stamp = ros::Time::now();

    traj_point.time_from_start = ros::Duration(dur);
    traj.points.push_back(traj_point);

    return true;
  }

  // warning! this function is blocking (on time value)
  void waitAndPublishTf(double time, BodyKinematics::BodyPose pose,
                        urdf::Pose travel_st)
  {
    const double tf_pub_period = time / TF_QUANTITY_PER_STEP;

    for (std::size_t i = 0; i != TF_QUANTITY_PER_STEP; ++i)
    {
      publishTransformToBase(pose);
      publishTransformToWorld(travel_st);
      shiftBodyPose(pose, tf_pub_period);
      shiftTravelState(travel_st, tf_pub_period);
      ros::Duration(tf_pub_period).sleep();
    }
  }

  // publish TF, is used for visualization.
  void publishTransformToBase(const BodyKinematics::BodyPose& pose)
  {
    tf::Transform transform;
    tf::Quaternion q;

    double roll, pitch, yaw;
    pose.rotation.getRPY(roll, pitch, yaw);

    transform.setOrigin(tf::Vector3(0, 0, 0));
    q.setRPY(-roll, -pitch, -yaw);
    transform.setRotation(q);

    tf_br_.sendTransform(tf::StampedTransform(
        transform, ros::Time::now(), "body_link", "center_of_rotation"));

    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    transform.setOrigin(tf::Vector3(-pose.position.x, -pose.position.y,
                                    body_.getClearance() - pose.position.z));
    tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                              "center_of_rotation", "base"));
  }

  // publish TF, is used for visualization.
  void publishTransformToWorld(const urdf::Pose& travel_st)
  {
    tf::Transform transform;
    tf::Quaternion q;

    double roll, pitch, yaw;
    travel_st.rotation.getRPY(roll, pitch, yaw);

    transform.setOrigin(tf::Vector3(0, 0, 0));
    q.setRPY(0, 0, -yaw);
    transform.setRotation(q);

    tf_br_.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "base", "rot_base"));

    transform.setOrigin(
        tf::Vector3(-travel_st.position.x, -travel_st.position.y, 0));
    q.setRPY(0, 0, 0);
    transform.setRotation(q);

    tf_br_.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "rot_base", "world"));
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  BodyKinematics body_;
  GaitGenerator gait_;
  BodyAutoLevelingState auto_lvl_state_;

  ros::Subscriber gait_vel_sub_;
  ros::Subscriber body_vel_sub_;
  ros::Subscriber legs_vel_sub_;
  ros::Subscriber cmd_sub_;
  ros::Publisher jnt_traj_pub_;

  geometry_msgs::Twist gait_velocity_;
  geometry_msgs::Twist body_velocity_;
  goliath_msgs::LegsVel legs_velocity_;

  // used only for velocities reset
  ros::Time last_gait_or_legs_command_time_;
  ros::Time last_body_command_time_;

  // current state of robot
  BodyKinematics::LegsStance curr_legs_stance_;
  BodyKinematics::BodyPose curr_body_pose_;
  urdf::Pose curr_travel_state_;

  // used for visualisation
  tf::TransformBroadcaster tf_br_;
};

const double GoliathMotion::MOVE_TIME_STEP = 0.1;
const double GoliathMotion::VEL_RESET_TIMEOUT = 0.5;

int main(int argc, char** argv)
{
  // uncomment for debug purpose
  // sleep(2);

  // Let's start ROS!
  ros::init(argc, argv, "goliath_motion");
  ros::NodeHandle node;

  GoliathMotion goliath_motion(node);

  goliath_motion.spin();

  return 0;
}
