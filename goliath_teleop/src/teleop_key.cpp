#include <stdio.h>
#include <termios.h> //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>  //STDIN_FILENO
#include "ros/ros.h"
#include "goliath_msgs/Pose.h"
#include <sstream>
#include "signal.h"

class GoliathTeleop
{
public:
  GoliathTeleop(ros::NodeHandle node) : n_(node)
  {
    pose_pub_ = n_.advertise<goliath_msgs::Pose>("leg_position", 10);
    pos_.number_of_leg = DEFAULT_LEG_NUMB;
    pos_.position_of_leg.x = pos_.position_of_leg.y =
    pos_.position_of_leg.z = DEFAULT_LEG_COORD;

    switchConsoleBuffState(false);

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

    switch (c = getchar())
    {
    case 'a':
      pos_.position_of_leg.x -= STEP;
      break;
    case 'd':
      pos_.position_of_leg.x += STEP;
      break;
    case 's':
      pos_.position_of_leg.y -= STEP;
      break;
    case 'w':
      pos_.position_of_leg.y += STEP;
      break;
    case 'r':
      pos_.position_of_leg.z -= STEP;
      break;
    case 'f':
      pos_.position_of_leg.z += STEP;
      break;
    default:
      if (int(c) > '0' && int(c) <= '0' + MAX_LEG_NUMB)
        pos_.number_of_leg = int(c) - '0';
      else
        ready_to_pub = false;
      break;
    }

    if (ready_to_pub)
    {
      ROS_INFO_STREAM(std::endl
                      << pos_);
      pose_pub_.publish(pos_);
    }
  }

private:
  static const double STEP = 0.01;
  static const int DEFAULT_LEG_NUMB = 1;
  static const int MAX_LEG_NUMB = 6;
  static const double DEFAULT_LEG_COORD = 0.05;

  ros::NodeHandle n_;
  ros::Publisher pose_pub_;
  goliath_msgs::Pose pos_;

  // disable or enable buffering input
  void switchConsoleBuffState(bool state)
  {
    static struct termios oldt, newt;
    if (!state)
    {
      /*tcgetattr gets the parameters of the current terminal
      STDIN_FILENO will tell tcgetattr that it should write the settings
      of stdin to oldt*/
      tcgetattr(STDIN_FILENO, &oldt);
      /*now the settings will be copied*/
      newt = oldt;

      /*ICANON normally takes care that one line at a time will be processed
      that means it will return if it sees a "\n" or an EOF or an EOL*/
      newt.c_lflag &= ~(ICANON | ECHO);

      /*For non-blocking getchar*/
      newt.c_cc[VMIN] = newt.c_cc[VTIME] = 0;

      /*Those new settings will be set to STDIN
      TCSANOW tells tcsetattr to change attributes immediately. */
      tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    }
    else
      /*restore the old settings*/
      tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  }
};

int main(int argc, char* argv[])
{
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
