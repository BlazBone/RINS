#include "ros/ros.h"
#include "my_package/Sum.h"
#include <cstdlib>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_service_client");
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<my_package::Sum>("my_service");
  my_package::Sum srv;

  for (int i = 0; i < 10; i++)
  {
    srv.request.nums.clear();
    for (int j = 0; j < 10; j++)
    {
      srv.request.nums.push_back(rand() % 100);
    }
    if (client.call(srv))
    {
      ROS_INFO("Sequence:");
      for (int j = 0; j < 10; j++)
      {
        ROS_INFO("%d", srv.request.nums[j]);
      }
      ROS_INFO("Sum: %d", srv.response.sum);
    }
    else
    {
      ROS_ERROR("Failed to call service my_service");
      return 1;
    }
  }

  return 0;
}
