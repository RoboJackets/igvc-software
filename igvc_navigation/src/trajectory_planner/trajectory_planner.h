#ifndef PROJECT_PATH_FOLLOWER_H
#define PROJECT_PATH_FOLLOWER_H

#include <mutex>

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
  void publishDebug(const igvc_msgs::trajectoryConstPtr& trajectory);

  /**
   * Returns the smoothed path from SmoothControl
   * @return a trajectory, if there exists a path and a waypoint
   */
  std::optional<igvc_msgs::trajectoryPtr> getSmoothPath();

  nav_msgs::PathConstPtr getPatchedPath(const nav_msgs::PathConstPtr& msg) const;

  ros::Publisher cmd_pub_;
  ros::Publisher target_pub_;
  ros::Publisher trajectory_pub_;
  ros::Publisher smoothed_pub_;
  ros::Publisher debug_pub_;

  nav_msgs::PathConstPtr path_;
  geometry_msgs::PointStampedConstPtr waypoint_;

  RobotState state_;
  ros::Time last_time_;
  ros::Time last_path_time_;
  double path_timeout_;

  double loop_hz_{};
  double stop_dist_{};
  double maximum_vel_{};
  double end_distance_threshold{};

  std::mutex path_mutex_;
  std::mutex state_mutex_;

  std::unique_ptr<SmoothControl> controller_;
  std::unique_ptr<MotionProfiler> motion_profiler_;
};
#endif  // PROJECT_PATH_FOLLOWER_H
