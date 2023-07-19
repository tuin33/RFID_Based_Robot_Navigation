#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "set_permissions_node");

  ros::NodeHandle nh("~");

  std::string command;
  nh.param<std::string>("command", command, "");

  if (command.empty())
  {
    ROS_ERROR("Command parameter is empty!");
    return 1;
  }

  int result = system(command.c_str());
  if (result == -1)
  {
    ROS_ERROR("Failed to execute command: %s", command.c_str());
    return 1;
  }

  ROS_INFO("Command executed successfully: %s", command.c_str());
  return 0;
}