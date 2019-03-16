#ifndef PROJECT_PATH_FOLLOWER_H
#define PROJECT_PATH_FOLLOWER_H

#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include "smooth_control.h"

class PathFollower
{
public:
  PathFollower();

private:
  void pathCallback(const nav_msgs::PathConstPtr& msg);
  void waypointCallback(const geometry_msgs::PointStampedConstPtr& msg);
  void positionCallback(const nav_msgs::OdometryConstPtr& msg);

  ros::Publisher cmd_pub_;
  ros::Publisher target_pub_;
  ros::Publisher trajectory_pub_;

  nav_msgs::PathConstPtr path_;
  geometry_msgs::PointStampedConstPtr waypoint_;

  double stop_dist_{};
  double maximum_vel_{};

  std::unique_ptr<SmoothControl> controller_;
};
#endif  // PROJECT_PATH_FOLLOWER_H
