#include "ros/ros.h"
#include "my_package/Sum.h"

bool computeSum(my_package::Sum::Request &req, my_package::Sum::Response &res)
{
  int sum = 0;
  for (int i = 0; i < req.nums.size(); i++)
  {
    sum += req.nums[i];
  }
  res.sum = sum;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_service_node");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("my_service", computeSum);
  ROS_INFO("Ready to compute sum.");

  ros::spin();

  return 0;
}