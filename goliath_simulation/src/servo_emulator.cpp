#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <string>

class ServoEmulator
{
public:
  ServoEmulator(ros::NodeHandle nh) : nh_(nh)
  {
    ros::NodeHandle priv_nh("~");
    std::string ctrl_name;

    priv_nh.getParam("controller_name", ctrl_name);

    jnt_traj_sub_ = nh_.subscribe(ctrl_name + "/command", TRAJ_QUEUE_SIZE,
                                  &ServoEmulator::jntTrajectoryCallback, this);

    jnt_st_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);
  }

private:

  void jntTrajectoryCallback(const trajectory_msgs::JointTrajectory& jnt_traj)
  {
    sensor_msgs::JointState jnt_st;
    static std::vector<double> prev_st;

    jnt_st.name = jnt_traj.joint_names;

    ros::Rate loop_rate(RATE_VAL);

    for (auto p : jnt_traj.points)
    {
      std::size_t itterations_numb = p.time_from_start.toSec() * RATE_VAL;
      std::vector<double> delta_pos;

      for (std::size_t j = 0; j != p.positions.size(); j++)
      {
        delta_pos.push_back(
            (p.positions[j] - (prev_st.empty() ? 0 : prev_st[j])) /
            itterations_numb);
      }

      for (std::size_t i = 1; i <= itterations_numb; i++)
      {
        loop_rate.sleep();
        jnt_st.header.stamp = ros::Time::now();

        for (std::size_t j = 0; j != p.positions.size(); j++)
          jnt_st.position.push_back((prev_st.empty() ? 0 : prev_st[j]) +
                                    i * delta_pos[j]);

        jnt_st_pub_.publish(jnt_st);
        jnt_st.position.clear();
      }

      jnt_st.header.stamp = ros::Time::now();
      prev_st = jnt_st.position = p.positions;

      jnt_st_pub_.publish(jnt_st);
      jnt_st.position.clear();
    }
  }

  static const double RATE_VAL;
  static const std::size_t TRAJ_QUEUE_SIZE = 100;
  ros::NodeHandle nh_;
  ros::Subscriber jnt_traj_sub_;
  ros::Publisher jnt_st_pub_;
};

const double ServoEmulator::RATE_VAL = 20;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "servo_emulator");
  ros::NodeHandle nh;

  ServoEmulator servo_emulator(nh);
  ros::spin();

  return 0;
}
