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

    printInstruction();
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

  static const KeyJoystick KEY_JOY_MOVE;
  static const KeyJoystick KEY_JOY_MOVE_FAST;
  static const KeyJoystick KEY_JOY_ROT_BODY;
  static const KeyJoystick KEY_JOY_TR_BODY;
  static const std::array<KeyCode, DUMMY_MODE> KEYCODES_MODE;
  static const KeyCode KEYCODE_LEG_SEL;
  static const KeyCode KEYCODE_HELP;

  friend std::ostream& operator<<(std::ostream& os, const KeyJoystick& kj)
  {
    std::string sp("  ");
    os << sp << kj.turn_l << sp << kj.forw << sp << kj.turn_r << sp << endl
       << sp << kj.left << sp << kj.backw << sp << kj.right << sp << endl;
    return os;
  }

  void printInstruction()
  {
    std::string mode_msg;
    static const std::array<std::string, DUMMY_MODE> mode_names{
        "Single leg mode", "Tripod walking mode", "Wave walking mode",
        "Ripple walking mode"};

    std::string leg_sel_keycode(1, KEYCODE_LEG_SEL); // KEYCODE_LEG_SEL;
    if (KEYCODE_LEG_SEL == '\t')
      leg_sel_keycode = "Tab";

    for (std::size_t m = SINGLE_LEG_MODE; m != DUMMY_MODE; m++)
      mode_msg += "  " + mode_names[m] + " - " + KEYCODES_MODE[m] + '\n';

    ROS_INFO_STREAM(endl
                    << "Keyboard teleop controller for Goliath robot." << endl
                    << endl
                    << "To select a mode, use keys:" << endl
                    << mode_msg << endl
                    << "For legs and movement control:" << endl
                    << KEY_JOY_MOVE << endl
                    << "For faster speed: " << endl
                    << KEY_JOY_MOVE_FAST << endl
                    << "To control the rotation of the body:" << endl
                    << KEY_JOY_ROT_BODY << endl
                    << "To control the translation of the body:" << endl
                    << KEY_JOY_TR_BODY << endl
                    << "In " << mode_names[SINGLE_LEG_MODE] << " use key "
                    << leg_sel_keycode << " for leg change." << endl
                    << endl
                    << "Press " << KEYCODE_HELP
                    << " to print this instuction again." << endl
                    << "Press Ctrl+c to exit.");
  }

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
    double koef = 1;

    if (kc == KEY_JOY_MOVE_FAST.forw || kc == KEY_JOY_MOVE.forw)
    {
      koef = kc == KEY_JOY_MOVE.forw ? 1 : 2;
      gait_vel.linear.x = koef * speed.linear;
      leg_vel.x = koef * speed.linear;
    }
    else if (kc == KEY_JOY_MOVE_FAST.backw || kc == KEY_JOY_MOVE.backw)
    {
      koef = kc == KEY_JOY_MOVE.backw ? 1 : 2;
      gait_vel.linear.x = -koef * speed.linear;
      leg_vel.x = -koef * speed.linear;
    }
    else if (kc == KEY_JOY_MOVE_FAST.left || kc == KEY_JOY_MOVE.left)
    {
      koef = kc == KEY_JOY_MOVE.left ? 1 : 2;
      gait_vel.linear.y = koef * speed.linear;
      leg_vel.y = koef * speed.linear;
    }
    else if (kc == KEY_JOY_MOVE_FAST.right || kc == KEY_JOY_MOVE.right)
    {
      koef = kc == KEY_JOY_MOVE.right ? 1 : 2;
      gait_vel.linear.y = -koef * speed.linear;
      leg_vel.y = -koef * speed.linear;
    }
    else if (kc == KEY_JOY_MOVE_FAST.turn_l || kc == KEY_JOY_MOVE.turn_l)
    {
      koef = kc == KEY_JOY_MOVE.turn_l ? 1 : 2;
      gait_vel.angular.z = koef * speed.angular;
      leg_vel.z = koef * speed.linear;
    }
    else if (kc == KEY_JOY_MOVE_FAST.turn_r || kc == KEY_JOY_MOVE.turn_r)
    {
      koef = kc == KEY_JOY_MOVE.turn_r ? 1 : 2;
      gait_vel.angular.z = -koef * speed.angular;
      leg_vel.z = -koef * speed.linear;
    }
    else if (kc == KEYCODE_LEG_SEL || kc == KEYCODES_MODE[SINGLE_LEG_MODE])
    {
      koef = 0.2;
      leg_vel.z = koef * speed.linear;
    }
    else
      return false;

    return true;
  }

  bool updateBodyVel(KeyCode kc, geometry_msgs::Twist& body_vel)
  {
    double koef = 1;

    if (kc == KEY_JOY_TR_BODY.forw || kc == KEY_JOY_TR_BODY.backw)
    {
      koef = (kc == KEY_JOY_TR_BODY.forw) ? 1 : -1;
      body_vel.linear.x = koef * BODY_SPEED.linear;
    }
    else if (kc == KEY_JOY_TR_BODY.left || kc == KEY_JOY_TR_BODY.right)
    {
      koef = (kc == KEY_JOY_TR_BODY.left) ? 1 : -1;
      body_vel.linear.y = koef * BODY_SPEED.linear;
    }
    else if (kc == KEY_JOY_TR_BODY.turn_l || kc == KEY_JOY_TR_BODY.turn_r)
    {
      koef = (kc == KEY_JOY_TR_BODY.turn_l) ? 1 : -1;
      body_vel.linear.z = koef * BODY_SPEED.linear;
    }
    else if (kc == KEY_JOY_ROT_BODY.forw || kc == KEY_JOY_ROT_BODY.backw)
    {
      koef = (kc == KEY_JOY_ROT_BODY.forw) ? 1 : -1;
      body_vel.angular.y = koef * BODY_SPEED.angular;
    }
    else if (kc == KEY_JOY_ROT_BODY.left || kc == KEY_JOY_ROT_BODY.right)
    {
      koef = (kc == KEY_JOY_ROT_BODY.left) ? 1 : -1;
      body_vel.angular.x = -koef * BODY_SPEED.angular;
    }
    else if (kc == KEY_JOY_ROT_BODY.turn_l || kc == KEY_JOY_ROT_BODY.turn_r)
    {
      koef = (kc == KEY_JOY_ROT_BODY.turn_l) ? 1 : -1;
      body_vel.angular.z = koef * BODY_SPEED.angular;
    }
    else
      return false;

    return true;
  }

  bool updateMode(KeyCode kc, TeleopMode& mode)
  {
    for (std::size_t m = SINGLE_LEG_MODE; m != DUMMY_MODE; ++m)
    {
      if (kc == KEYCODES_MODE[m])
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

    if (kc == KEYCODE_LEG_SEL && mode_ == SINGLE_LEG_MODE)
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

    if(kc == KEYCODE_HELP)
      printInstruction();
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

const TeleopKey::KeyJoystick TeleopKey::KEY_JOY_MOVE{'w', 's', 'a',
                                                     'd', 'q', 'e'};
const TeleopKey::KeyJoystick TeleopKey::KEY_JOY_MOVE_FAST{'W', 'S', 'A',
                                                          'D', 'Q', 'E'};
const TeleopKey::KeyJoystick TeleopKey::KEY_JOY_ROT_BODY{'i', 'k', 'j',
                                                         'l', 'u', 'o'};
const TeleopKey::KeyJoystick TeleopKey::KEY_JOY_TR_BODY{'I', 'K', 'J',
                                                        'L', 'U', 'O'};
const std::array<TeleopKey::KeyCode, TeleopKey::DUMMY_MODE>
    TeleopKey::KEYCODES_MODE{{'`', '1', '2', '3'}};
const TeleopKey::KeyCode TeleopKey::KEYCODE_LEG_SEL = '\t';
const TeleopKey::KeyCode TeleopKey::KEYCODE_HELP = 'h';

int main(int argc, char* argv[])
{
  // let's star
  ros::init(argc, argv, "teleop");
  ros::NodeHandle n;

  TeleopKey teleop(n);
  teleop.spin();

  return 0;
}
