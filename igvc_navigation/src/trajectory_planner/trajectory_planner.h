#ifndef PROJECT_PATH_FOLLOWER_H
#define PROJECT_PATH_FOLLOWER_H

#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <igvc_utils/robot_state.h>

#include "smooth_control.h"

class TrajectoryPlanner
{
public:
  TrajectoryPlanner();

private:
  void pathCallback(const nav_msgs::PathConstPtr& msg);
  void waypointCallback(const geometry_msgs::PointStampedConstPtr& msg);
  void positionCallback(const nav_msgs::OdometryConstPtr& msg);
  void encoderCallback(const igvc_msgs::velocity_pairConstPtr& msg);
  void updateTrajectory();

  void publishTrajectory(const igvc_msgs::trajectoryConstPtr& trajectory);

  /**
   * Returns
   * @return
   */
  std::optional<igvc_msgs::trajectoryPtr> getSmoothPath();

  void publishTarget(const RobotState& target);
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

  std::unique_ptr<SmoothControl> controller_;
  std::unique_ptr<MotionProfiler> motion_profiler_;
};
#endif  // PROJECT_PATH_FOLLOWER_H
