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
  if (enabled) {
  	  ROS_INFO_STREAM("The robot is enabled. Data will be written");
  } else { 
	  ROS_INFO_STREAM("The robot is disabled. Data will not be written.");
  }
}

void motors_callback(const igvc_msgs::velocity_pair& msg) {
  if(enabled) {
  	ROS_INFO_STREAM("Data stored");
    file << ("%.20f", msg.header.stamp.toSec()) << ", ";
    file << msg.left_velocity << ", " << msg.right_velocity << ", " << std::endl;
  } else {
    ROS_INFO_STREAM("Robot disabled, not writing data");
  }
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "pidtester");

  ros::NodeHandle nh;

  ros::Subscriber enabled_sub = nh.subscribe("/robot_enabled", 1, enabled_callback);

  ros::Subscriber motors_sub = nh.subscribe("/motors", 1, motors_callback);

  file.open("/home/robojackets/Desktop/speed_data.csv");
  file << "timestamp, left_velocity, right_velocity" << std::endl;


/*
  while (!enabled)d
  {
    ros::spinOnce();
  }
  */

  //usleep(500);

  ros::spin();

  return 0;
}
