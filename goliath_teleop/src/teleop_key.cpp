#include <stdio.h>
#include <termios.h> //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>  //STDIN_FILENO
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "goliath_msgs/LegsVel.h"
#include "goliath_msgs/MotionCmd.h"
using std::endl;

// class contains instruments for remote control Goliath robot.
// it use a keyboard for send BodyPose and LegsPosition message to topic
class TeleopKey
{
public:
  TeleopKey(ros::NodeHandle node)
      : nh_(node), mode_(TRIPOD_MODE), single_leg_number_(0)
  {
    // init publishers
    legs_vel_pub_ =
        nh_.advertise<goliath_msgs::LegsVel>("legs_cmd_vel", PUB_QUEUE_SZ);
    body_vel_pub_ =
        nh_.advertise<geometry_msgs::Twist>("body_cmd_vel", PUB_QUEUE_SZ);
    gait_vel_pub_ =
        nh_.advertise<geometry_msgs::Twist>("gait_cmd_vel", PUB_QUEUE_SZ);
    motion_cmd_pub_ =
        nh_.advertise<goliath_msgs::MotionCmd>("motion_cmd", PUB_QUEUE_SZ);

    ROS_INFO_STREAM("Place for description!");
    switchConsoleBuffState(false);
  }

  ~TeleopKey() { switchConsoleBuffState(true); }

  void spin()
  {
    ros::Rate loop_rate(RATE_VAL);

    while (ros::ok())
    {
      keyboardPoll();
      loop_rate.sleep();
      ros::spinOnce();
    }
  }

private:
  struct Speed
  {
    double linear;
    double angular;
  };

  static const std::size_t MAX_LEG_NUMBER = 6;
  static const int RATE_VAL = 50;
  static const Speed SINGLE_LEG_SPEED;
  static const Speed TRIPOD_SPEED;
  static const Speed WAVE_SPEED;
  static const Speed RIPPLE_SPEED;
  static const Speed BODY_SPEED;

  enum QueueSize
  {
    PUB_QUEUE_SZ = 10
  };

  enum TeleopMode
  {
    SINGLE_LEG_MODE,
    TRIPOD_MODE,
    WAVE_MODE,
    RIPPLE_MODE
  };

  void keyboardPoll()
  {
    geometry_msgs::Twist gait_vel, body_vel;
    goliath_msgs::LegsVel legs_vel;

    bool ready_to_pub_leg = false;
    bool ready_to_pub_body = false;
    bool ready_to_pub_gait = false;
    bool ready_to_pub_cmd = false;
    int c;

    Speed walk_speed;
    if (mode_ == TRIPOD_MODE)
      walk_speed = TRIPOD_SPEED;
    if (mode_ == WAVE_MODE)
      walk_speed = WAVE_SPEED;
    if (mode_ == RIPPLE_MODE)
      walk_speed = RIPPLE_SPEED;

    // use low-level function getchar (cin doesn't work with unbuffered input)
    switch (c = getchar())
    {
    case 'w':
    case 'W':
      if (mode_ != SINGLE_LEG_MODE)
      {
        gait_vel.linear.x = (c == 'W' ? 2 : 1) * walk_speed.linear;
        ready_to_pub_gait = true;
      }
      else
      {
        legs_vel.velocities[single_leg_number_].x =
            (c == 'W' ? 2 : 1) * SINGLE_LEG_SPEED.linear;
        ready_to_pub_leg = true;
      }

      break;

    case 's':
    case 'S':
      if (mode_ != SINGLE_LEG_MODE)
      {
        gait_vel.linear.x = (c == 'S' ? 2 : 1) * -walk_speed.linear;
        ready_to_pub_gait = true;
      }
      else
      {
        legs_vel.velocities[single_leg_number_].x =
            (c == 'S' ? 2 : 1) * -SINGLE_LEG_SPEED.linear;
        ready_to_pub_leg = true;
      }
      break;

    case 'd':
    case 'D':
      if (mode_ != SINGLE_LEG_MODE)
      {
        gait_vel.linear.y = (c == 'D' ? 2 : 1) * -walk_speed.linear;
        ready_to_pub_gait = true;
      }
      else
      {
        legs_vel.velocities[single_leg_number_].y =
            (c == 'D' ? 2 : 1) * -SINGLE_LEG_SPEED.linear;
        ready_to_pub_leg = true;
      }
      break;

    case 'a':
    case 'A':
      if (mode_ != SINGLE_LEG_MODE)
      {
        gait_vel.linear.y = (c == 'A' ? 2 : 1) * walk_speed.linear;
        ready_to_pub_gait = true;
      }
      else
      {
        legs_vel.velocities[single_leg_number_].y =
            (c == 'A' ? 2 : 1) * SINGLE_LEG_SPEED.linear;
        ready_to_pub_leg = true;
      }
      break;

    case 'q':
    case 'Q':
      if (mode_ != SINGLE_LEG_MODE)
      {
        gait_vel.angular.z = (c == 'Q' ? 2 : 1) * walk_speed.angular;
        ready_to_pub_gait = true;
      }
      else
      {
        legs_vel.velocities[single_leg_number_].z =
            (c == 'Q' ? 2 : 1) * SINGLE_LEG_SPEED.linear;
        ready_to_pub_leg = true;
      }
      break;

    case 'e':
    case 'E':
      if (mode_ != SINGLE_LEG_MODE)
      {
        gait_vel.angular.z = (c == 'E' ? 2 : 1) * -walk_speed.angular;
        ready_to_pub_gait = true;
      }
      else
      {
        legs_vel.velocities[single_leg_number_].z =
            (c == 'E' ? 2 : 1) * -SINGLE_LEG_SPEED.linear;
        ready_to_pub_leg = true;
      }
      break;

    case '`':
      mode_ = SINGLE_LEG_MODE;
      single_leg_number_ = 0;
      legs_vel.velocities[single_leg_number_].z = SINGLE_LEG_SPEED.linear / 5;
      ready_to_pub_leg = true;
      break;

    case '1':
      mode_ = TRIPOD_MODE;
      ready_to_pub_cmd = true;
      break;

    case '2':
      mode_ = WAVE_MODE;
      ready_to_pub_cmd = true;
      break;

    case '3':
      mode_ = RIPPLE_MODE;
      ready_to_pub_cmd = true;
      break;

    case '\t':
      if (mode_ == SINGLE_LEG_MODE)
      {
        if (single_leg_number_++ == MAX_LEG_NUMBER - 1)
          single_leg_number_ = 0;

        legs_vel.velocities[single_leg_number_].z = SINGLE_LEG_SPEED.linear / 5;
        ready_to_pub_leg = true;
      }
      break;

    case 'i':
      body_vel.angular.y = BODY_SPEED.angular;
      ready_to_pub_body = true;
      break;

    case 'k':
      body_vel.angular.y = -BODY_SPEED.angular;
      ready_to_pub_body = true;
      break;

    case 'l':
      body_vel.angular.x = BODY_SPEED.angular;
      ready_to_pub_body = true;
      break;

    case 'j':
      body_vel.angular.x = -BODY_SPEED.angular;
      ready_to_pub_body = true;
      break;

    case 'u':
      body_vel.angular.z = BODY_SPEED.angular;
      ready_to_pub_body = true;
      break;

    case 'o':
      body_vel.angular.z = -BODY_SPEED.angular;
      ready_to_pub_body = true;
      break;

    case 'I':
      body_vel.linear.x = BODY_SPEED.linear;
      ready_to_pub_body = true;
      break;

    case 'K':
      body_vel.linear.x = -BODY_SPEED.linear;
      ready_to_pub_body = true;
      break;

    case 'L':
      body_vel.linear.y = BODY_SPEED.linear;
      ready_to_pub_body = true;
      break;

    case 'J':
      body_vel.linear.y = -BODY_SPEED.linear;
      ready_to_pub_body = true;
      break;

    case 'U':
      body_vel.linear.z = BODY_SPEED.linear;
      ready_to_pub_body = true;
      break;

    case 'O':
      body_vel.linear.z = -BODY_SPEED.linear;
      ready_to_pub_body = true;
      break;

    default:

      break;
    }

    if (ready_to_pub_gait)
      gait_vel_pub_.publish(gait_vel);
    if (ready_to_pub_body)
      body_vel_pub_.publish(body_vel);
    if (ready_to_pub_cmd)
    {
      goliath_msgs::MotionCmd msg;
      msg.type = mode_;
      motion_cmd_pub_.publish(msg);
    }
    if (ready_to_pub_leg)
      legs_vel_pub_.publish(legs_vel);
  }

  // disable or enable buffering input
  // without this method you have to press "enter" key after each button
  void switchConsoleBuffState(bool state)
  {
    static struct termios oldt, newt;
    if (!state)
    {
      // tcgetattr gets the parameters of the current terminal
      // STDIN_FILENO will tell tcgetattr that it should write the settings
      // of stdin to oldt
      tcgetattr(STDIN_FILENO, &oldt);
      // now the settings will be copied
      newt = oldt;

      // ICANON normally takes care that one line at a time will be processed
      // that means it will return if it sees a "\n" or an EOF or an EOL
      newt.c_lflag &= ~(ICANON | ECHO);

      // For non-blocking getchar
      newt.c_cc[VMIN] = newt.c_cc[VTIME] = 0;

      // Those new settings will be set to STDIN
      // TCSANOW tells tcsetattr to change attributes immediately.
      tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    }
    else
      // restore the old settings
      tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  }

  ros::NodeHandle nh_;
  ros::Publisher legs_vel_pub_;
  ros::Publisher body_vel_pub_;
  ros::Publisher gait_vel_pub_;
  ros::Publisher motion_cmd_pub_;

  TeleopMode mode_;
  std::size_t single_leg_number_;
};

const TeleopKey::Speed TeleopKey::SINGLE_LEG_SPEED{0.05, 0};
const TeleopKey::Speed TeleopKey::TRIPOD_SPEED{0.06, 0.25};
const TeleopKey::Speed TeleopKey::WAVE_SPEED{0.015, 0.1};
const TeleopKey::Speed TeleopKey::RIPPLE_SPEED{0.035, 0.2};
const TeleopKey::Speed TeleopKey::BODY_SPEED{0.04, 0.35};

int main(int argc, char* argv[])
{
  // let's star
  ros::init(argc, argv, "teleop");
  ros::NodeHandle n;

  TeleopKey teleop(n);
  teleop.spin();

  return 0;
}
