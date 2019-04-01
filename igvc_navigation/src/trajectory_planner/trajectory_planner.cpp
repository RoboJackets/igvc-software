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
#include <igvc_utils/NodeUtils.hpp>

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

  igvc::param(pNh, "axle_length", axle_length, 0.52);

  igvc::param(pNh, "smooth_control/k1", smooth_control_options.k1, 1.0);
  igvc::param(pNh, "smooth_control/k2", smooth_control_options.k2, 3.0);

  igvc::param(pNh, "path_generation/simulation_frequency", path_generation_options.simulation_frequency, 100.0);
  igvc::param(pNh, "path_generation/simulation_horizon", path_generation_options.simulation_horizon, 5.0);
  igvc::param(pNh, "path_generation/velocity", path_generation_options.simulation_velocity, 1.0);

  igvc::param(pNh, "target_selection/lookahead_dist", target_selection_options.lookahead_dist, 2.0);
  igvc::param(pNh, "target_selection/reached_distance", target_selection_options.target_reached_distance, 0.05);
  igvc::param(pNh, "target_selection/move_threshold", target_selection_options.target_move_threshold, 0.05);

  igvc::param(pNh, "curvature_blending/blending_distance", curvature_blending_options.blending_distance, 1.0);

  if (path_generation_options.simulation_frequency <= 0)
  {
    ROS_WARN_STREAM("Simulation frequency (currently " << path_generation_options.simulation_frequency
                                                       << ") should be greater than 0. Setting to 1 for now.");
    path_generation_options.simulation_frequency = 1;
  }
  controller_ = std::unique_ptr<SmoothControl>(new SmoothControl{ smooth_control_options, path_generation_options,
                                                                  target_selection_options, curvature_blending_options,
                                                                  axle_length });

  // load global parameters
  igvc::getParam(pNh, "maximum_vel", maximum_vel_);
  igvc::param(pNh, "stop_dist", stop_dist_, 0.9);

  ros::Subscriber path_sub = nh.subscribe("/path", 1, &TrajectoryPlanner::pathCallback, this);
  ros::Subscriber encoder_sub = nh.subscribe("/encoders", 1, &TrajectoryPlanner::encoderCallback, this);
  ros::Subscriber pose_sub = nh.subscribe("/odometry/filtered", 1, &TrajectoryPlanner::positionCallback, this);
  ros::Subscriber waypoint_sub = nh.subscribe("/waypoint", 1, &TrajectoryPlanner::waypointCallback, this);

  cmd_pub_ = nh.advertise<igvc_msgs::velocity_pair>("/motors", 1);
  target_pub_ = nh.advertise<geometry_msgs::PointStamped>("/target_point", 1);
  trajectory_pub_ = nh.advertise<nav_msgs::Path>("/trajectory", 1);
  smoothed_pub_ = nh.advertise<nav_msgs::Path>("/smoothed", 1);

  ros::spin();
}

void TrajectoryPlanner::pathCallback(const nav_msgs::PathConstPtr& msg)
{
  ROS_DEBUG_STREAM("Follower got path. Size: " << msg->poses.size());
  //  path_ = msg;
  // TODO: Remove patch when motion planning correctly incorporates heading
  path_ = getPatchedPath(msg);
  updateTrajectory();
  smoothed_pub_.publish(path_);
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
  updateTrajectory();
}

nav_msgs::PathConstPtr TrajectoryPlanner::getPatchedPath(const nav_msgs::PathConstPtr& msg) const
{
  // Patch heading into each point
  nav_msgs::Path new_path = *msg;
  size_t num_poses = new_path.poses.size();
  for (size_t i = 0; i < num_poses; i++)
  {
    double delta_x = new_path.poses[i + 1].pose.position.x - new_path.poses[i].pose.position.x;
    double delta_y = new_path.poses[i + 1].pose.position.y - new_path.poses[i].pose.position.y;
    double heading = atan2(delta_y, delta_x);
    new_path.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(heading);
  }
  new_path.poses.back().pose.orientation = new_path.poses[num_poses - 2].pose.orientation;

  // Get points between each point to smooth it out
  nav_msgs::PathPtr smoothed_path = boost::make_shared<nav_msgs::Path>();
  smoothed_path->poses.emplace_back(new_path.poses.front());
  for (size_t i = 0; i < num_poses - 1; i++)
  {
    double mid_x = (new_path.poses[i].pose.position.x + new_path.poses[i + 1].pose.position.x) / 2;
    double mid_y = (new_path.poses[i].pose.position.y + new_path.poses[i + 1].pose.position.y) / 2;
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = mid_x;
    pose.pose.position.y = mid_y;
    smoothed_path->poses.emplace_back(pose);
  }
  smoothed_path->poses.emplace_back(new_path.poses.back());

  // Patch heading
  for (size_t i = 0; i < smoothed_path->poses.size(); i++)
  {
    double delta_x = smoothed_path->poses[i + 1].pose.position.x - smoothed_path->poses[i].pose.position.x;
    double delta_y = smoothed_path->poses[i + 1].pose.position.y - smoothed_path->poses[i].pose.position.y;
    double heading = atan2(delta_y, delta_x);
    smoothed_path->poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(heading);
  }
  smoothed_path->poses.back().pose.orientation = smoothed_path->poses[num_poses - 2].pose.orientation;
  smoothed_path->header = msg->header;
  return smoothed_path;
}

void TrajectoryPlanner::encoderCallback(const igvc_msgs::velocity_pairConstPtr& msg)
{
  state_.setVelocity(msg);
  last_time_ = msg->header.stamp;
  updateTrajectory();
}

void TrajectoryPlanner::updateTrajectory()
{
  std::optional<igvc_msgs::trajectoryPtr> trajectory = getSmoothPath();

  if (trajectory && trajectory.value().get() != nullptr)
  {
    motion_profiler::profileTrajectory(*trajectory);
    publishTrajectory(*trajectory);
  }
}

/**
 * Thread that generates the trajectories using the SmoothController
 * Generate Path:
 * getClosestIndex
 * get closest & second closest target
 * get action from current position based on target using curvature blending
 * propogate state
 * add to path
 *
 * Three pass:
 * 1: v = min(v, thing)
 * 2: forward pass
 * 3: backward pass
 *
 * publish trajectory
 */
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

  igvc_msgs::trajectoryPtr trajectory;

  RobotState target;
  controller_->getPath(path_, trajectory, state_);

  publishTarget(target);
  ROS_DEBUG_STREAM_THROTTLE(1, "Distance to target: " << state_.distTo(target) << " (m)");
  return trajectory;
}

void TrajectoryPlanner::publishTarget(const RobotState& target)
{
  geometry_msgs::PointStamped target_point;
  target_point.header.frame_id = "/odom";
  target_point.header.stamp = last_time_;
  target_point.point.x = target.x;
  target_point.point.y = target.y;
  target_pub_.publish(target_point);
}

void TrajectoryPlanner::publishTrajectory(const igvc_msgs::trajectoryConstPtr& trajectory)
{
  trajectory_pub_.publish(trajectory);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_planner");
  TrajectoryPlanner path_follower;
  return 0;
}
