#include <igvc_msgs/velocity_pair.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <chrono>
#include <fstream>

bool enabled = false;
std::ofstream file;


void enabled_callback(const std_msgs::BoolConstPtr& msg)
{
  enabled = msg->data;
}

void motors_callback(const igvc_msgs::velocity_pair& msg) {
  //save to file??
  if(enabled) {
    file << msg.left_velocity << ", " << msg.right_velocity << ", ";
    file << msg.header.stamp.toSec() << std::endl;
  }
  ROS_INFO_STREAM("ITS WORKING");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pidtester");

  ros::NodeHandle nh;

  ros::Subscriber enabled_sub = nh.subscribe("/robot_enabled", 1, enabled_callback);

  ros::Subscriber motors_sub = nh.subscribe("/motors", 1, motors_callback);

  file.open("./speed_data.csv");


/*
  while (!enabled)
  {
    ros::spinOnce();
  }
  */

  //usleep(500);

  ros::spin();

  return 0;
}
