#include "ros/ros.h"
#include "my_package/MyMessage.h"

int main(int argc, char **argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "publisher");

  // Create a node handle
  ros::NodeHandle n;

  // Create a publisher object for the "my_topic" topic
  ros::Publisher pub = n.advertise<my_package::MyMessage>("my_topic", 100);


  int id_counter = 1;
  // Loop at 10 Hz and publish the message
  ros::Rate rate(10);
  while (ros::ok())
  {
      // Create a message object
    my_package::MyMessage msg;
    msg.message = "Hello, world!";
    msg.id = id_counter++;

    pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}