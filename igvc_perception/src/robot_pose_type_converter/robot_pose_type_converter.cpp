#include "robot_pose_type_converter.h"

RobotPoseTypeConveter::RobotPoseTypeConveter()
{
  private_nh_ = ros::NodeHandle("~");
  pose_publisher_ =
      private_nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/odometry/pose_with_cov_stamped", 1);
  pose_subscriber_ = private_nh_.subscribe("/odometry/filtered", 1, &RobotPoseTypeConveter::poseCallback, this);
}

void RobotPoseTypeConveter::poseCallback(const nav_msgs::Odometry& odometry)
{
  geometry_msgs::PoseWithCovarianceStamped message;
  message.header = odometry.header;
  message.pose = odometry.pose;
  pose_publisher_.publish(message);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_pose_type_converter");
  RobotPoseTypeConveter rptc = RobotPoseTypeConveter();
  ros::spin();
}
