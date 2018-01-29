#include <stdio.h>
#include <termios.h> //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>  //STDIN_FILENO
#include "ros/ros.h"
#include "goliath_msgs/Pose.h"
#include <sstream>
#include "signal.h"

// class contains instruments for remote control Goliath robot.
// it use a keyboard for send Pose message to topic
class GoliathTeleop
{
public:
  GoliathTeleop(ros::NodeHandle node) : n_(node), leg_numb_(0)
  {
    pose_pub_ = n_.advertise<goliath_msgs::Pose>("leg_positions", 10);
    switchConsoleBuffState(false);

    // init position of legs
    geometry_msgs::Point32 point1;
    point1.x = DEFAULT_LEG_COORDS[0][0];
    point1.y = DEFAULT_LEG_COORDS[0][1];
    point1.z = DEFAULT_LEG_COORDS[0][2];
    geometry_msgs::Point32 point2;
    point2.x = DEFAULT_LEG_COORDS[1][0];
    point2.y = DEFAULT_LEG_COORDS[1][1];
    point2.z = DEFAULT_LEG_COORDS[1][2];

    std::size_t sz = pos_.position_of_legs.size();
    for (std::size_t i = 0; i != sz; ++i)
      pos_.position_of_legs[i] = (i < sz / 2) ? point1 : point2;

    ROS_INFO_STREAM("Goliath leg position teleop."
                    << std::endl
                    << "Use keys 's','w','a','d','r','f' for x,y,z transfer."
                    << std::endl
                    << "Use keys '1'..'6' for leg's choice" << std::endl
                    << "Start value:" << std::endl
                    << pos_ << std::endl);
  }

  ~GoliathTeleop() { switchConsoleBuffState(true); }

  void keyboardPoll()
  {
    bool ready_to_pub = true;
    int c;

    //use low-level function getchar (cin doesn't work with unbuffered input)
    switch (c = getchar())
    {
    case 'a':
      pos_.position_of_legs[leg_numb_].x -= STEP;
      break;
    case 'd':
      pos_.position_of_legs[leg_numb_].x += STEP;
      break;
    case 's':
      pos_.position_of_legs[leg_numb_].y -= STEP;
      break;
    case 'w':
      pos_.position_of_legs[leg_numb_].y += STEP;
      break;
    case 'r':
      pos_.position_of_legs[leg_numb_].z -= STEP;
      break;
    case 'f':
      pos_.position_of_legs[leg_numb_].z += STEP;
      break;
    default:
      // change leg's number if digit is pressed
      if (int(c) > '0' && int(c) <= '0' + MAX_LEG_NUMB)
        leg_numb_ = int(c) - 1 - '0';
      else
        ready_to_pub = false;
      break;
    }

    if (ready_to_pub)
    {
      ROS_INFO_STREAM(std::endl << "Leg's number: " <<
                      (leg_numb_ + 1) << std::endl
                      << pos_.position_of_legs[leg_numb_]);
      pose_pub_.publish(pos_);
    }
  }

private:
  //iteration of Pose, when press the key
  static const double STEP;
  static const int MAX_LEG_NUMB = 6;
  //first index - side of legs (0/1 - left or right)
  //second index - location of legs (0 - front, 1 - middle, 2 - rear)
  static const double DEFAULT_LEG_COORDS[2][3];

  ros::NodeHandle n_;
  ros::Publisher pose_pub_;
  goliath_msgs::Pose pos_;
  std::size_t leg_numb_;

  //disable or enable buffering input
  //without this method you have to press "enter" key after each button
  void switchConsoleBuffState(bool state)
  {
    static struct termios oldt, newt;
    if (!state)
    {
      //tcgetattr gets the parameters of the current terminal
      //STDIN_FILENO will tell tcgetattr that it should write the settings
      //of stdin to oldt
      tcgetattr(STDIN_FILENO, &oldt);
      //now the settings will be copied
      newt = oldt;

      //ICANON normally takes care that one line at a time will be processed
      //that means it will return if it sees a "\n" or an EOF or an EOL
      newt.c_lflag &= ~(ICANON | ECHO);

      //For non-blocking getchar
      newt.c_cc[VMIN] = newt.c_cc[VTIME] = 0;

      //Those new settings will be set to STDIN
      //TCSANOW tells tcsetattr to change attributes immediately.
      tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    }
    else
      //restore the old settings
      tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  }
};

const double GoliathTeleop::STEP = 0.01;
const double GoliathTeleop::DEFAULT_LEG_COORDS[2][3] = {{-0.08, 0.00, 0.05},
                                                      {0.08, 0.00, 0.05}};

int main(int argc, char* argv[])
{
  //let's star
  ros::init(argc, argv, "teleop");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  GoliathTeleop goliath_teleop(n);

  while (ros::ok())
  {
    goliath_teleop.keyboardPoll();
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
