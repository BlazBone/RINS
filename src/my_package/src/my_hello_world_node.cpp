#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_hello_world_node");
  ros::NodeHandle nh;
  ROS_INFO("Hello from ROS!");
  return 0;
}
