#ifndef PROJECT_PATH_FOLLOWER_H
#define PROJECT_PATH_FOLLOWER_H

#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include "Smooth_control.h"

class Path_follower
{
public:
  Path_follower();

private:
  void path_callback(const nav_msgs::PathConstPtr& msg);
  void waypoint_callback(const geometry_msgs::PointStampedConstPtr& msg);
  void position_callback(const nav_msgs::OdometryConstPtr& msg);

  ros::Publisher cmd_pub;
  ros::Publisher target_pub;
  ros::Publisher trajectory_pub;

  nav_msgs::PathConstPtr path;
  geometry_msgs::PointStampedConstPtr waypoint;

  double stop_dist{};
  double maximum_vel{};

  std::unique_ptr<Smooth_control> controller;
};
#endif  // PROJECT_PATH_FOLLOWER_H
