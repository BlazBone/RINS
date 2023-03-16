#include "ros/ros.h"
#include "my_package/MyMessage.h"

void callback(const my_package::MyMessage::ConstPtr& msg)
{
  ROS_INFO("Received message: [%s], ID: [%d]", msg->message.c_str(), msg->id);
}

int main(int argc, char **argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "subscriber");

  // Create a node handle
  ros::NodeHandle n;

  // Create a subscriber object for the "my_topic" topic and register a callback function
  ros::Subscriber sub = n.subscribe("my_topic", 10, callback);

  // Spin and wait for incoming messages
  ros::spin();

  return 0;
}
