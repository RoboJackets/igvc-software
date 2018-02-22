#include <igvc_msgs/velocity_pair.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <chrono>
#include <fstream>

bool enabled = false;
std::ofstream file;
float target_left_vel;
float target_right_vel;
ros::Time start_time;

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
  if (!enabled && ros::Time::now().sec <= 10000) {
    enabled = true;
  }
  if(enabled) {
  	target_left_vel = msg.left_velocity;
    target_right_vel = msg.right_velocity;
  } else {
    ROS_INFO_STREAM("Robot disabled, ignored /motors message");
  }
}

void encoders_callback(const igvc_msgs::velocity_pair& msg) {
  ROS_INFO_STREAM(ros::Time::now().sec);
  if(enabled) {
    ROS_INFO_STREAM("Data stored");
    ros::Duration time_diff = msg.header.stamp - start_time;
    file << time_diff.sec << "." << time_diff.nsec << ", ";
    file << msg.left_velocity << ", " << msg.right_velocity << ", ";
    file << target_left_vel << "," << target_right_vel << std::endl;
      } else {
        ROS_INFO_STREAM("Robot disabled, not writing data");
      }
    }
    
    
    
    int main(int argc, char** argv)
    {
      ros::init(argc, argv, "pidtester");"

  ros::NodeHandle nh;

  start_time = ros::Time::now();

  ros::Subscriber enabled_sub = nh.subscribe("/robot_enabled", 1, enabled_callback);

  ros::Subscriber motors_sub = nh.subscribe("/motors", 1, motors_callback);

  ros::Subscriber encoders_sub = nh.subscribe("/encoders", 1, encoders_callback);

  file.open("/home/robojackets/Desktop/speed_data.csv");
  file << "timestamp, left_velocity, right_velocity, target_left_velocity, target_right_velocity" << std::endl;


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
