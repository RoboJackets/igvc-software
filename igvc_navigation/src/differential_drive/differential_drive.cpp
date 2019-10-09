#include <parameter_assertions/assertions.h>
#include <ros/ros.h>

#include "differential_drive.h"

DifferentialDrive::DifferentialDrive()
{
  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  assertions::param(pNh, "axle_length", axle_length_, 0.48);
  assertions::param(pNh, "max_vel", max_vel_, 3.0);
  ros::Subscriber mbf_twist = nh.subscribe("/cmd_vel", 1, &DifferentialDrive::twistToVelocity, this);
  vel_pub_ = nh.advertise<igvc_msgs::velocity_pair>("/motors", 1);

  ros::spin();
}

void DifferentialDrive::twistToVelocity(geometry_msgs::Twist twist)
{
  double speed = twist.linear.x;
  double rotation = twist.angular.z;
  double vel_right = speed + (rotation * axle_length_) / 2;
  double vel_left = speed - (rotation * axle_length_) / 2;

  double max_calc_vel = fmax(vel_right, vel_left);
  if (max_calc_vel > max_vel_)
  {
    vel_right *= max_vel_ / max_calc_vel;
    vel_left *= max_vel_ / max_calc_vel;
  }

  igvc_msgs::velocity_pair vel_msg;
  vel_msg.right_velocity = vel_right;
  vel_msg.left_velocity = vel_left;
  vel_msg.duration = 0.02;
  vel_msg.header.stamp = ros::Time::now();
  vel_pub_.publish(vel_msg);
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "differential_drive");
  DifferentialDrive differential_drive;
}
