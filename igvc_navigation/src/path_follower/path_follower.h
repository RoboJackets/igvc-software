#ifndef PROJECT_PATH_FOLLOWER_H
#define PROJECT_PATH_FOLLOWER_H

#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <mutex>

#include "smooth_control.h"

class PathFollower
{
public:
  PathFollower();

private:
  void pathCallback(const nav_msgs::PathConstPtr& msg);
  void waypointCallback(const geometry_msgs::PointStampedConstPtr& msg);
  void positionCallback(const nav_msgs::OdometryConstPtr& msg);
  void encoderCallback(const igvc_msgs::velocity_pairConstPtr& msg);
  void trajectoryLoop(double loop_hz);
  nav_msgs::PathConstPtr getPatchedPath(const nav_msgs::PathConstPtr& msg) const;

  ros::Publisher cmd_pub_;
  ros::Publisher target_pub_;
  ros::Publisher trajectory_pub_;
  ros::Publisher smoothed_pub_;

  nav_msgs::PathConstPtr path_;
  geometry_msgs::PointStampedConstPtr waypoint_;

  RobotState state_;
  ros::Time last_time_;

  double stop_dist_{};
  double maximum_vel_{};
  std::mutex state_mutex_;

  std::unique_ptr<SmoothControl> controller_;
};
#endif  // PROJECT_PATH_FOLLOWER_H
