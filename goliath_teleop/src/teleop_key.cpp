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
      keyboardPollAndPublish();
      loop_rate.sleep();
      ros::spinOnce();
    }
  }

private:
  enum TeleopMode
  {
    SINGLE_LEG_MODE,
    TRIPOD_MODE,
    WAVE_MODE,
    RIPPLE_MODE,
    DUMMY_MODE
  };

  enum QueueSize
  {
    PUB_QUEUE_SZ = 10
  };

  typedef char KeyCode;
  // list of KEYCODES
  struct KeyJoystick
  {
    KeyCode forw;
    KeyCode backw;
    KeyCode left;
    KeyCode right;
    KeyCode turn_l;
    KeyCode turn_r;
  };

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

  static constexpr KeyJoystick KEY_JOY_MOVE{'w', 's', 'a', 'd', 'q', 'e'};
  static constexpr KeyJoystick KEY_JOY_MOVE_FAST{'W', 'S', 'A', 'D', 'Q', 'E'};
  static constexpr KeyJoystick KEY_JOY_ROT_BODY{'i', 'k', 'j', 'l', 'u', 'o'};
  static constexpr KeyJoystick KEY_JOY_TR_BODY{'I', 'K', 'J', 'L', 'U', 'O'};

  static const std::array<KeyCode, DUMMY_MODE> MODE_KEYCODES;
  static const KeyCode LEG_SEL_KEYCODE = '\t';

  goliath_msgs::MotionCmd createMotionCmdMsg(TeleopMode mode)
  {
    goliath_msgs::MotionCmd ret;
    switch (mode)
    {
    case TRIPOD_MODE:
      ret.type = goliath_msgs::MotionCmd::SELECT_TRIPOD_GAIT;
      break;

    case WAVE_MODE:
      ret.type = goliath_msgs::MotionCmd::SELECT_WAVE_GAIT;
      break;

    case RIPPLE_MODE:
      ret.type = goliath_msgs::MotionCmd::SELECT_RIPPLE_GAIT;
      break;

    default:
      break;
    }
    return ret;
  }

  bool updateGaitOrLegVel(KeyCode kc, geometry_msgs::Twist& gait_vel,
                          geometry_msgs::Vector3& leg_vel, Speed speed)
  {
    bool ret = true;
    int koef = 1;
    switch (kc)
    {
    case KEY_JOY_MOVE_FAST.forw:
      koef = 2;
    case KEY_JOY_MOVE.forw:
      gait_vel.linear.x = koef * speed.linear;
      leg_vel.x = koef * speed.linear;
      break;

    case KEY_JOY_MOVE_FAST.backw:
      koef = 2;
    case KEY_JOY_MOVE.backw:
      gait_vel.linear.x = -koef * speed.linear;
      leg_vel.x = -koef * speed.linear;
      break;

    case KEY_JOY_MOVE_FAST.left:
      koef = 2;
    case KEY_JOY_MOVE.left:
      gait_vel.linear.y = koef * speed.linear;
      leg_vel.y = koef * speed.linear;
      break;

    case KEY_JOY_MOVE_FAST.right:
      koef = 2;
    case KEY_JOY_MOVE.right:
      gait_vel.linear.y = -koef * speed.linear;
      leg_vel.y = -koef * speed.linear;
      break;

    case KEY_JOY_MOVE_FAST.turn_l:
      koef = 2;
    case KEY_JOY_MOVE.turn_l:
      gait_vel.angular.z = koef * speed.angular;
      leg_vel.z = koef * speed.linear;
      break;

    case KEY_JOY_MOVE_FAST.turn_r:
      koef = 2;
    case KEY_JOY_MOVE.turn_r:
      gait_vel.angular.z = -koef * speed.angular;
      leg_vel.z = -koef * speed.linear;
      break;

    default:
      if (kc == LEG_SEL_KEYCODE || kc == MODE_KEYCODES[SINGLE_LEG_MODE])
        leg_vel.z = koef * speed.linear / 5;
      else
        ret = false;
      break;
    }

    return ret;
  }

  bool updateBodyVel(KeyCode kc, geometry_msgs::Twist& body_vel)
  {
    bool ret = true;

    switch (kc)
    {
    case KEY_JOY_TR_BODY.forw:
      body_vel.linear.x = BODY_SPEED.linear;
      break;

    case KEY_JOY_TR_BODY.backw:
      body_vel.linear.x = -BODY_SPEED.linear;
      break;

    case KEY_JOY_TR_BODY.left:
      body_vel.linear.y = BODY_SPEED.linear;
      break;

    case KEY_JOY_TR_BODY.right:
      body_vel.linear.y = -BODY_SPEED.linear;
      break;

    case KEY_JOY_TR_BODY.turn_l:
      body_vel.linear.z = BODY_SPEED.linear;
      break;

    case KEY_JOY_TR_BODY.turn_r:
      body_vel.linear.z = -BODY_SPEED.linear;
      break;

    case KEY_JOY_ROT_BODY.forw:
      body_vel.angular.y = BODY_SPEED.angular;
      break;

    case KEY_JOY_ROT_BODY.backw:
      body_vel.angular.y = -BODY_SPEED.angular;
      break;

    case KEY_JOY_ROT_BODY.left:
      body_vel.angular.x = BODY_SPEED.angular;
      break;

    case KEY_JOY_ROT_BODY.right:
      body_vel.angular.x = -BODY_SPEED.angular;
      break;

    case KEY_JOY_ROT_BODY.turn_l:
      body_vel.angular.z = BODY_SPEED.angular;
      break;

    case KEY_JOY_ROT_BODY.turn_r:
      body_vel.angular.z = -BODY_SPEED.angular;
      break;

    default:
      ret = false;
      break;
    }
    return ret;
  }

  bool updateMode(KeyCode kc, TeleopMode& mode)
  {
    for (std::size_t m = SINGLE_LEG_MODE; m != DUMMY_MODE; ++m)
    {
      if (kc == MODE_KEYCODES[m])
      {
        mode = static_cast<TeleopMode>(m);
        return true;
      }
    }

    return false;
  }

  void keyboardPollAndPublish()
  {
    geometry_msgs::Twist gait_vel, body_vel;
    goliath_msgs::LegsVel legs_vel;

    // use low-level function getchar (cin doesn't work with unbuffered input)
    KeyCode kc = getchar();

    if (kc == LEG_SEL_KEYCODE && mode_ == SINGLE_LEG_MODE)
    {
      if (single_leg_number_++ == MAX_LEG_NUMBER - 1)
        single_leg_number_ = 0;
    }

    if (updateMode(kc, mode_))
      motion_cmd_pub_.publish(createMotionCmdMsg(mode_));

    if (updateBodyVel(kc, body_vel))
      body_vel_pub_.publish(body_vel);

    if (mode_ == SINGLE_LEG_MODE)
    {
      geometry_msgs::Twist dummy;
      if (updateGaitOrLegVel(kc, dummy, legs_vel.velocities[single_leg_number_],
                             SINGLE_LEG_SPEED))
        legs_vel_pub_.publish(legs_vel);
    }
    else
    {
      Speed walk_speed;

      if (mode_ == TRIPOD_MODE)
        walk_speed = TRIPOD_SPEED;
      if (mode_ == WAVE_MODE)
        walk_speed = WAVE_SPEED;
      if (mode_ == RIPPLE_MODE)
        walk_speed = RIPPLE_SPEED;

      geometry_msgs::Vector3 dummy;
      if (updateGaitOrLegVel(kc, gait_vel, dummy, walk_speed))
        gait_vel_pub_.publish(gait_vel);
    }
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

const std::array<TeleopKey::KeyCode, TeleopKey::DUMMY_MODE>
    TeleopKey::MODE_KEYCODES{{'`', '1', '2', '3'}};

int main(int argc, char* argv[])
{
  // let's star
  ros::init(argc, argv, "teleop");
  ros::NodeHandle n;

  TeleopKey teleop(n);
  teleop.spin();

  return 0;
}
