#include <stdio.h>
#include <termios.h> //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>  //STDIN_FILENO
#include "ros/ros.h"
#include "goliath_msgs/LegsPosition.h"
#include "goliath_msgs/BodyPose.h"
#include "geometry_msgs/Point32.h"
#include <iomanip>
using std::endl;

// class contains instruments for remote control Goliath robot.
// it use a keyboard for send BodyPose and LegsPosition message to topic
class TeleopKey
{
public:
  TeleopKey(ros::NodeHandle node) : n_(node), mode_(LEGS_MODE)
  {
    // init publishers
    legs_pos_pub_ = n_.advertise<goliath_msgs::LegsPosition>("legs_position",
                                                             LEGS_POS_QUEUE_SZ);
    body_pose_pub_ =
        n_.advertise<goliath_msgs::BodyPose>("body_pose", BODY_POSE_QUEUE_SZ);

    this->setLegsPosToDefault();
    this->setBodyPoseToDefault();

    ROS_INFO_STREAM(
        endl
        << "Goliath keyboard teleop." << endl
        << "Use keys 'b' and 'l' for select mode: body/legs." << endl
        << "In body and legs modes use arrows for XY transfer, '+' and '-' for "
           "Z transfer." << endl
        << "In body mode use 's','w','a','d','r','f' for RPY rotation." << endl
        << "In legs mode use keys '0'..'5' for leg's choice." << endl
        << "In all modes use 'o' for reset parameters." << endl
        << "Legs mode." << endl);
    switchConsoleBuffState(false);
  }

  ~TeleopKey() { switchConsoleBuffState(true); }

  void keyboardPoll()
  {
    bool ready_to_pub = true;
    int c;

    // use low-level function getchar (cin doesn't work with unbuffered input)
    switch (c = getchar())
    {
    case 'l':
      mode_ = LEGS_MODE;
      ROS_INFO_STREAM("Legs mode" << endl);
      this->setLegsPosToDefault();
      this->setBodyPoseToDefault();
      break;
    case 'b':
      mode_ = BODY_MODE;
      ROS_INFO_STREAM("Body mode" << endl);
      this->setLegsPosToDefault();
      this->setBodyPoseToDefault();
      break;
    case 68: // left arrow
      if (mode_ == LEGS_MODE)
        legs_pos_.position_of_legs[curr_leg_numb_].y += LEG_XYZ_STEP;
      else
        body_pose_.position.y += BODY_XYZ_STEP;
      break;
    case 67: // right arrow
      if (mode_ == LEGS_MODE)
        legs_pos_.position_of_legs[curr_leg_numb_].y -= LEG_XYZ_STEP;
      else
        body_pose_.position.y -= BODY_XYZ_STEP;
      break;
    case 65: // up arrow
      if (mode_ == LEGS_MODE)
        legs_pos_.position_of_legs[curr_leg_numb_].x += LEG_XYZ_STEP;
      else
        body_pose_.position.x += BODY_XYZ_STEP;
      break;
    case 66: // down arrow
      if (mode_ == LEGS_MODE)
        legs_pos_.position_of_legs[curr_leg_numb_].x -= LEG_XYZ_STEP;
      else
        body_pose_.position.x -= BODY_XYZ_STEP;
      break;
    case '+':
    case '=':
      if (mode_ == LEGS_MODE)
        legs_pos_.position_of_legs[curr_leg_numb_].z += LEG_XYZ_STEP;
      else
        body_pose_.position.z += BODY_XYZ_STEP;
      break;
    case '-':
      if (mode_ == LEGS_MODE)
        legs_pos_.position_of_legs[curr_leg_numb_].z -= LEG_XYZ_STEP;
      else
        body_pose_.position.z -= BODY_XYZ_STEP;
      break;
    case 'a':
      if (mode_ == BODY_MODE)
        body_pose_.roll -= BODY_RPY_STEP;
      else
        ready_to_pub = false;
      break;
    case 'd':
      if (mode_ == BODY_MODE)
        body_pose_.roll += BODY_RPY_STEP;
      else
        ready_to_pub = false;
      break;
    case 's':
      if (mode_ == BODY_MODE)
        body_pose_.pitch -= BODY_RPY_STEP;
      else
        ready_to_pub = false;
      break;
    case 'w':
      if (mode_ == BODY_MODE)
        body_pose_.pitch += BODY_RPY_STEP;
      else
        ready_to_pub = false;
      break;
    case 'r':
      if (mode_ == BODY_MODE)
        body_pose_.yaw -= BODY_RPY_STEP;
      else
        ready_to_pub = false;
      break;
    case 'f':
      if (mode_ == BODY_MODE)
        body_pose_.yaw += BODY_RPY_STEP;
      else
        ready_to_pub = false;
      break;
    case 'o':
      this->setLegsPosToDefault();
      this->setBodyPoseToDefault();
      break;
    default:
      // change leg's number if digit is pressed
      if (c >= '0' && c < '0' + MAX_LEG_NUMB)
      {
        curr_leg_numb_ = c - '0';
      }
      else
        ready_to_pub = false;
      break;
    }

    if (ready_to_pub)
    {
      std::streamsize prec = std::cout.precision();
      std::cout << std::setprecision(4);
      if (mode_ == BODY_MODE)
      {
        ROS_INFO_STREAM(endl
                        << body_pose_);
        body_pose_pub_.publish(body_pose_);
      }
      else if (mode_ == LEGS_MODE)
      {
        ROS_INFO_STREAM(endl
                        << legs_pos_);
        legs_pos_pub_.publish(legs_pos_);
      }
      std::cout << std::setprecision(prec);
    }
  }

private:
  // iteration of Pose, when press the key
  static const double LEG_XYZ_STEP, BODY_RPY_STEP, BODY_XYZ_STEP;
  static const int MAX_LEG_NUMB = 6;

  enum QueueSize
  {
    LEGS_POS_QUEUE_SZ = 50,
    BODY_POSE_QUEUE_SZ = 50
  };

  enum TeleopMode
  {
    LEGS_MODE,
    BODY_MODE
  };

  ros::NodeHandle n_;
  ros::Publisher body_pose_pub_;
  ros::Publisher legs_pos_pub_;
  goliath_msgs::LegsPosition legs_pos_;
  goliath_msgs::BodyPose body_pose_;
  TeleopMode mode_;
  std::size_t curr_leg_numb_;

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
  void setBodyPoseToDefault()
  {
    // init body position
    body_pose_.roll = body_pose_.pitch = body_pose_.yaw = 0;
    body_pose_.position.x = body_pose_.position.y = body_pose_.position.z = 0;
  }

  void setLegsPosToDefault()
  {
    // init leg position
    double x, y, z;

    z = -0.05;
    for (std::size_t i = 0; i != legs_pos_.position_of_legs.size(); ++i)
    {
      switch (i)
      {
      case 0:
        x = 0.05, y = 0.05;
        break;
      case 1:
        x = 0, y = 0.05;
        break;
      case 2:
        x = -0.05, y = 0.05;
        break;
      case 3:
        x = 0.05, y = -0.05;
        break;
      case 4:
        x = 0, y = -0.05;
        break;
      case 5:
        x = -0.05, y = -0.05;
        break;
      default:
        break;
      }
      legs_pos_.position_of_legs[i].x = x;
      legs_pos_.position_of_legs[i].y = y;
      legs_pos_.position_of_legs[i].z = z;
    }
  }
};

const double TeleopKey::LEG_XYZ_STEP = 0.01;
const double TeleopKey::BODY_RPY_STEP = 3.1416 / 100.0;
const double TeleopKey::BODY_XYZ_STEP = 0.01;

int main(int argc, char* argv[])
{
  // let's star
  ros::init(argc, argv, "teleop");
  ros::NodeHandle n;
  ros::Rate loop_rate(50);

  TeleopKey teleop(n);

  while (ros::ok())
  {
    teleop.keyboardPoll();
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
