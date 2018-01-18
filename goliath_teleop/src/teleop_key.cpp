#include <stdio.h>
#include <termios.h> //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>  //STDIN_FILENO
#include "ros/ros.h"
#include "geometry_msgs/Point32.h"
#include <sstream>

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
    newt.c_lflag &= ~(ICANON|ECHO);

    /*Those new settings will be set to STDIN
    TCSANOW tells tcsetattr to change attributes immediately. */
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  }
  else
    /*restore the old settings*/
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "teleop");

  float coord[3];
  if (argc != 4)
  {
    ROS_ERROR_STREAM("Enter X Y Z coord!");
    return -1;
  }
  else
  {
    std::stringstream ss;

    for (int i = 0; i != 3; i++)
    {
      ss << argv[i+1];
      ss >> coord[i];
      ss.str("");
      ss.clear();
    }
  }

  ROS_INFO_STREAM("Simple position teleop.\n"
                  << "Use keys 's','w','a','d','r','f' for x y z transfer.");
  ros::NodeHandle n;
  ros::Publisher position_pub =
      n.advertise<geometry_msgs::Point32>("position", 100);
  ros::Rate loop_rate(10);

  int c;
  geometry_msgs::Point32 pos;
  pos.x = coord[0], pos.y = coord[1], pos.z = coord[2];
  ROS_INFO_STREAM(std::endl << pos);
  const float step = 0.01;

  switchConsoleBuffState(false);
  while ((c = getchar()) != EOF || ros::ok())
  {
    bool ready_to_pub = true;

    switch (c)
    {
    case 'a':
      pos.x -= step;
      ready_to_pub = true;
      break;
    case 'd':
      pos.x += step;
      break;
    case 's':
      pos.y -= step;
      break;
    case 'w':
      pos.y += step;
      break;
    case 'r':
      pos.z -= step;
      break;
    case 'f':
      pos.z += step;
      break;
    default:
      ready_to_pub = false;
      break;
    }

    if(ready_to_pub)
    {
      ROS_INFO_STREAM(std::endl << pos);
      position_pub.publish(pos);
    }

    loop_rate.sleep();
    ros::spinOnce();
  }
  switchConsoleBuffState(true);

  return 0;
}
