#ifndef SRC_ROBOT_POSE_TYPE_CONVERTER_H
#define SRC_ROBOT_POSE_TYPE_CONVERTER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

class RobotPoseTypeConveter
{
public:
  RobotPoseTypeConveter();

private:
  ros::NodeHandle private_nh_;
  ros::Publisher pose_publisher_;
  ros::Subscriber pose_subscriber_;

  void poseCallback(const nav_msgs::Odometry& odometry);
};

#endif  // SRC_ROBOT_POSE_TYPE_CONVERTER_H
