#include <igvc_msgs/velocity_pair.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <chrono>
#include <fstream>

bool enabled = false;

std::ofstream file;

void encoder_callback(const igvc_msgs::velocity_pairConstPtr& msg)
{
  file << msg->left_velocity << ", " << msg->right_velocity << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "encrecord");

  ros::NodeHandle nh;

  ros::Subscriber encoder_sub = nh.subscribe("/encoders", 10, encoder_callback);

  file.open("/home/robojackets/Desktop/data.csv");

  ros::spin();

  return 0;
}
