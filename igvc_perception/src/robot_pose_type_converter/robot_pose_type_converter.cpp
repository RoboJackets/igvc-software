#include "robot_pose_type_converter.h"

RobotPoseTypeConveter::RobotPoseTypeConveter()
{
  private_nh = ros::NodeHandle("~");
  pose_publisher = private_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/odometry/pose_with_cov_stamped", 1);
  pose_subscriber = private_nh.subscribe("/odometry/filtered", 1, &RobotPoseTypeConveter::poseCallback, this);
}

void RobotPoseTypeConveter::poseCallback(const nav_msgs::Odometry& odometry)
{
  geometry_msgs::PoseWithCovarianceStamped message;
  message.header = odometry.header;
  message.pose = odometry.pose;
  pose_publisher.publish(message);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_pose_type_converter");
  RobotPoseTypeConveter rptc();
  ros::spin();
}
