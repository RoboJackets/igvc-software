#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <thread>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_datatypes.h>

#include <igvc_msgs/trajectory.h>
#include <parameter_assertions/assertions.h>

#include "motion_profiler.h"
#include "trajectory_planner.h"

TrajectoryPlanner::TrajectoryPlanner()
{
  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  double axle_length;
  SmoothControlOptions smooth_control_options{};
  PathGenerationOptions path_generation_options{};
  TargetSelectionOptions target_selection_options{};
  CurvatureBlendingOptions curvature_blending_options{};

  WheelConstraint wheel_constraints{};
  RobotConstraint robot_constraints{};
  MotionProfilerOptions motion_profiler_options{};
  double target_velocity;

  using namespace assertions;
  Asserter asserter;

  asserter.param(pNh, "axle_length", axle_length, 0.52);
  asserter.param(pNh, "path_timeout", path_timeout_, 3.0);
  asserter.param(pNh, "node/loop_hz", loop_hz_, 20.0);

  asserter.param(pNh, "smooth_control/k1", smooth_control_options.k1, 1.0);
  asserter.param(pNh, "smooth_control/k2", smooth_control_options.k2, 3.0);

  asserter.param(pNh, "path_generation/simulation_frequency", path_generation_options.simulation_frequency, 100.0);
  asserter.param(pNh, "path_generation/simulation_horizon", path_generation_options.simulation_horizon, 5.0);
  asserter.param(pNh, "path_generation/velocity", path_generation_options.simulation_velocity, 1.0);

  asserter.param(pNh, "target_selection/lookahead_dist", target_selection_options.lookahead_dist, 2.0);
  asserter.param(pNh, "target_selection/reached_distance", target_selection_options.target_reached_distance, 0.05);
  asserter.param(pNh, "target_selection/move_threshold", target_selection_options.target_move_threshold, 0.05);

  asserter.param(pNh, "curvature_blending/blending_distance", curvature_blending_options.blending_distance, 1.0);

  asserter.param(pNh, "wheel_constraints/velocity", wheel_constraints.velocity, 3.0);
  asserter.param(pNh, "wheel_constraints/acceleration", wheel_constraints.acceleration, 5.0);
  asserter.param(pNh, "robot_constraints/velocity", robot_constraints.velocity, 2.0);
  asserter.param(pNh, "robot_constraints/acceleration", robot_constraints.acceleration, 5.0);

  asserter.param(pNh, "motion_profiler/target_velocity", target_velocity, 1.0);
  asserter.param(pNh, "motion_profiler/beta", motion_profiler_options.beta, 2.0);
  asserter.param(pNh, "motion_profiler/lambda", motion_profiler_options.lambda, 1.0);

  asserter.getParam(pNh, "end_distance_threshold", end_distance_threshold);

  double linear_acceleration_curvature_threshold;
  asserter.param(pNh, "linear_velocity_curvature_threshold", linear_acceleration_curvature_threshold, 1e-2);

  if (path_generation_options.simulation_frequency <= 0)
  {
    ROS_WARN_STREAM("Simulation frequency (currently " << path_generation_options.simulation_frequency
                                                       << ") should be greater than 0. Setting to 1 for now.");
    path_generation_options.simulation_frequency = 1;
  }
  controller_ = std::make_unique<SmoothControl>(smooth_control_options, path_generation_options,
                                                target_selection_options, curvature_blending_options, axle_length);
  motion_profiler_ =
      std::make_unique<MotionProfiler>(axle_length, wheel_constraints, robot_constraints, motion_profiler_options,
                                       target_velocity, linear_acceleration_curvature_threshold);

  // load global parameters
  asserter.param(pNh, "stop_dist", stop_dist_, 0.9);

  ros::Subscriber path_sub = nh.subscribe("/path", 1, &TrajectoryPlanner::pathCallback, this);
  ros::Subscriber encoder_sub = nh.subscribe("/encoders", 1, &TrajectoryPlanner::encoderCallback, this);
  ros::Subscriber pose_sub = nh.subscribe("/odometry/filtered", 1, &TrajectoryPlanner::positionCallback, this);
  ros::Subscriber waypoint_sub = nh.subscribe("/waypoint", 1, &TrajectoryPlanner::waypointCallback, this);

  cmd_pub_ = nh.advertise<igvc_msgs::velocity_pair>("/motors", 1);
  target_pub_ = nh.advertise<geometry_msgs::PointStamped>("/target_point", 1);
  trajectory_pub_ = nh.advertise<igvc_msgs::trajectory>("/trajectory", 1);
  smoothed_pub_ = nh.advertise<nav_msgs::Path>("/smoothed", 1);
  debug_pub_ = nh.advertise<nav_msgs::Path>("/trajectory_planner/debug", 1);

  std::thread trajectory_plan_thread(&TrajectoryPlanner::updateTrajectory, this);

  ros::spin();
  ROS_INFO("Shutting down...");
  trajectory_plan_thread.join();
}

void TrajectoryPlanner::pathCallback(const nav_msgs::PathConstPtr& msg)
{
  ROS_DEBUG_STREAM("Follower got path. Size: " << msg->poses.size());
  nav_msgs::PathConstPtr path = getPatchedPath(msg);
  smoothed_pub_.publish(path);
  std::lock_guard<std::mutex> path_mutex(path_mutex_);
  path_ = path;
  last_path_time_ = ros::Time::now();
}

void TrajectoryPlanner::waypointCallback(const geometry_msgs::PointStampedConstPtr& msg)
{
  waypoint_ = msg;
}

/**
Constructs a new trajectory to follow using the current path msg and publishes
the first velocity command from this trajectory.
*/
void TrajectoryPlanner::positionCallback(const nav_msgs::OdometryConstPtr& msg)
{
  state_.setState(msg);
  last_time_ = msg->header.stamp;
}

nav_msgs::PathConstPtr TrajectoryPlanner::getPatchedPath(const nav_msgs::PathConstPtr& msg) const
{
  // Get points between each point to smooth it out
  nav_msgs::PathPtr smoothed_path = boost::make_shared<nav_msgs::Path>();
  smoothed_path->poses.emplace_back(msg->poses.front());
  for (size_t i = 0; i < msg->poses.size() - 1; i++)
  {
    double mid_x = (msg->poses[i].pose.position.x + msg->poses[i + 1].pose.position.x) / 2;
    double mid_y = (msg->poses[i].pose.position.y + msg->poses[i + 1].pose.position.y) / 2;
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = mid_x;
    pose.pose.position.y = mid_y;
    smoothed_path->poses.emplace_back(pose);
  }
  smoothed_path->poses.emplace_back(msg->poses.back());

  // Patch heading
  for (size_t i = 0; i < smoothed_path->poses.size(); i++)
  {
    double delta_x = smoothed_path->poses[i + 1].pose.position.x - smoothed_path->poses[i].pose.position.x;
    double delta_y = smoothed_path->poses[i + 1].pose.position.y - smoothed_path->poses[i].pose.position.y;
    double heading = atan2(delta_y, delta_x);
    smoothed_path->poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(heading);
  }
  smoothed_path->poses.back().pose.orientation = smoothed_path->poses[msg->poses.size() - 2].pose.orientation;
  smoothed_path->header = msg->header;
  return smoothed_path;
}

void TrajectoryPlanner::encoderCallback(const igvc_msgs::velocity_pairConstPtr& msg)
{
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  state_.setVelocity(msg);
  last_time_ = msg->header.stamp;
}

void TrajectoryPlanner::updateTrajectory()
{
  ros::Rate rate(loop_hz_);
  while (ros::ok())
  {
    std::optional<igvc_msgs::trajectoryPtr> trajectory = getSmoothPath();
    if ((ros::Time::now() - last_path_time_).toSec() > path_timeout_)
    {
      ROS_INFO_STREAM_THROTTLE(1, "TIMEOUT ON TRAJ PLANNER " << (ros::Time::now() - last_path_time_).toSec());
      igvc_msgs::trajectoryPtr empty_traj = boost::make_shared<igvc_msgs::trajectory>();
      empty_traj->header.stamp = ros::Time::now();
      empty_traj->header.frame_id = "/odom";
      publishTrajectory(empty_traj);
    }

    if (trajectory && trajectory.value().get() != nullptr)
    {
      publishDebug(*trajectory);
      motion_profiler_->profileTrajectory(*trajectory, state_);
      publishTrajectory(*trajectory);
    }
  }
}

std::optional<igvc_msgs::trajectoryPtr> TrajectoryPlanner::getSmoothPath()
{
  if (waypoint_.get() == nullptr)
  {
    ROS_INFO_THROTTLE(1, "Waypoint empty.");
    return std::nullopt;
  }
  if (path_.get() == nullptr || path_->poses.empty() || path_->poses.size() < 2)
  {
    ROS_INFO_THROTTLE(1, "Path empty.");
    return std::nullopt;
  }

  double goal_dist = state_.distTo(waypoint_->point.x, waypoint_->point.y);

  ROS_DEBUG_STREAM_THROTTLE(1, "Distance to waypoint: " << goal_dist << "(m.)");

  igvc_msgs::trajectoryPtr trajectory = boost::make_shared<igvc_msgs::trajectory>();

  std::lock_guard<std::mutex> state_lock(state_mutex_);
  std::lock_guard<std::mutex> path_lock(path_mutex_);
  controller_->getPath(path_, trajectory, state_, end_distance_threshold);
  return trajectory;
}

void TrajectoryPlanner::publishTrajectory(const igvc_msgs::trajectoryConstPtr& trajectory)
{
  trajectory_pub_.publish(trajectory);
}

void TrajectoryPlanner::publishDebug(const igvc_msgs::trajectoryConstPtr& trajectory)
{
  igvc_msgs::trajectory_point point = trajectory->trajectory.front();
  nav_msgs::Path path;
  for (const auto& point : trajectory->trajectory)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose = point.pose;
    pose.header.stamp = point.header.stamp;
    pose.header.frame_id = "/odom";

    path.poses.emplace_back(pose);
  }

  path.header.stamp = trajectory->header.stamp;
  path.header.frame_id = "/odom";
  debug_pub_.publish(path);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_planner");
  TrajectoryPlanner path_follower;
  return 0;
}
