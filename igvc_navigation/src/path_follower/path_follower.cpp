#define _USE_MATH_DEFINES

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <igvc_msgs/velocity_pair.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_datatypes.h>

#include <igvc_utils/NodeUtils.hpp>
#include <igvc_utils/RobotState.hpp>
#include <thread>

#include "path_follower.h"

PathFollower::PathFollower()
{
  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  // load controller parameters
  using seconds = double;

  double target_velocity;
  double axle_length;
  double k1;
  double k2;
  double simulation_frequency;
  double lookahead_dist;
  double target_reached_distance;
  double target_move_threshold;
  double acceleration_threshold;
  double loop_hz;
  seconds simulation_horizon;
  igvc::param(pNh, "target_v", target_velocity, 1.0);
  igvc::param(pNh, "axle_length", axle_length, 0.52);
  igvc::param(pNh, "k1", k1, 1.0);
  igvc::param(pNh, "k2", k2, 3.0);
  igvc::param(pNh, "simulation_frequency", simulation_frequency, 100.0);
  igvc::param(pNh, "lookahead_dist", lookahead_dist, 2.0);
  igvc::param(pNh, "simulation_horizon", simulation_horizon, 5.0);
  igvc::param(pNh, "target_reached_distance", target_reached_distance, 0.05);
  igvc::param(pNh, "target_move_threshold", target_move_threshold, 0.05);
  igvc::param(pNh, "acceleration_threshold", acceleration_threshold, 1.0);
  igvc::param(pNh, "loop_hz", loop_hz, 20.0);
  if (simulation_frequency <= 0)
  {
    ROS_WARN_STREAM("Simulation frequency (currently " << simulation_frequency
                                                       << ") should be greater than 0. Setting to 1 for now.");
    simulation_frequency = 1;
  }
  controller_ = std::unique_ptr<SmoothControl>(
      new SmoothControl{ k1, k2, axle_length, simulation_frequency, target_velocity, lookahead_dist, simulation_horizon,
                         target_reached_distance, target_move_threshold, acceleration_threshold });

  // load global parameters
  igvc::getParam(pNh, "maximum_vel", maximum_vel_);
  igvc::param(pNh, "stop_dist", stop_dist_, 0.9);

  ros::Subscriber path_sub = nh.subscribe("/path", 1, &PathFollower::pathCallback, this);
  ros::Subscriber encoder_sub = nh.subscribe("/encoders", 1, &PathFollower::encoderCallback, this);
  ros::Subscriber pose_sub = nh.subscribe("/odometry/filtered", 1, &PathFollower::positionCallback, this);
  ros::Subscriber waypoint_sub = nh.subscribe("/waypoint", 1, &PathFollower::waypointCallback, this);

  cmd_pub_ = nh.advertise<igvc_msgs::velocity_pair>("/motors", 1);
  target_pub_ = nh.advertise<geometry_msgs::PointStamped>("/target_point", 1);
  trajectory_pub_ = nh.advertise<nav_msgs::Path>("/trajectory", 1);
  smoothed_pub_ = nh.advertise<nav_msgs::Path>("/smoothed", 1);

  std::thread trajectory_thread(&PathFollower::trajectoryLoop, this, loop_hz);
  ros::spin();
}

void PathFollower::pathCallback(const nav_msgs::PathConstPtr& msg)
{
  ROS_DEBUG_STREAM("Follower got path. Size: " << msg->poses.size());
  //  path_ = msg;
  // TODO: Remove patch when motion planning correctly incorporates heading
  path_ = getPatchedPath(msg);
  smoothed_pub_.publish(path_);
}

void PathFollower::waypointCallback(const geometry_msgs::PointStampedConstPtr& msg)
{
  waypoint_ = msg;
}

/**
Constructs a new trajectory to follow using the current path msg and publishes
the first velocity command from this trajectory.
*/
void PathFollower::positionCallback(const nav_msgs::OdometryConstPtr& msg)
{
  std::lock_guard<std::mutex> guard(state_mutex_);
  state_.setState(msg);
  last_time_ = msg->header.stamp;
}

nav_msgs::PathConstPtr PathFollower::getPatchedPath(const nav_msgs::PathConstPtr& msg) const
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

void PathFollower::encoderCallback(const igvc_msgs::velocity_pairConstPtr& msg)
{
  std::lock_guard<std::mutex> guard(state_mutex_);
  state_.setVelocity(msg);
  last_time_ = msg->header.stamp;
}

/**
 * Thread that generates the trajectories using the SmoothController
 */
void PathFollower::trajectoryLoop(double loop_hz)
{
  ros::Rate rate(loop_hz);
  while (ros::ok())
  {
    if (path_.get() == nullptr || path_->poses.empty() || path_->poses.size() < 2)
    {
      ROS_INFO_THROTTLE(1, "Path empty.");
      igvc_msgs::velocity_pair vel;
      vel.left_velocity = 0.;
      vel.right_velocity = 0.;
      cmd_pub_.publish(vel);
    }
    else
    {
      double goal_dist = state_.distTo(waypoint_->point.x, waypoint_->point.y);

      ROS_DEBUG_STREAM("Distance to waypoint: " << goal_dist << "(m.)");

      igvc_msgs::velocity_pair vel;  // immediate velocity command
      vel.header.stamp = last_time_;

      if (goal_dist <= stop_dist_)
      {
        // Stop when the robot is a set distance away from the waypoint
        ROS_INFO_STREAM_THROTTLE(0.5, ">>>WAYPOINT REACHED...STOPPING<<<");
        vel.right_velocity = 0;
        vel.left_velocity = 0;
      }
      else
      {
        nav_msgs::Path trajectory_msg;
        trajectory_msg.header.stamp = last_time_;
        trajectory_msg.header.frame_id = "/odom";

        RobotState target;
        controller_->getTrajectory(vel, path_, trajectory_msg, state_, target);

        trajectory_pub_.publish(trajectory_msg);

        // publish target position
        geometry_msgs::PointStamped target_point;
        target_point.header.frame_id = "/odom";
        target_point.header.stamp = last_time_;
        target_point.point.x = target.x;
        target_point.point.y = target.y;
        target_pub_.publish(target_point);

        ROS_DEBUG_STREAM_THROTTLE(1, "Distance to target: " << state_.distTo(target) << " (m)");
      }

      // make sure maximum velocity not exceeded
      double vehicle_velocity = (vel.left_velocity + vel.right_velocity) / 2;
      if (std::abs(vehicle_velocity) > maximum_vel_)
      {
        ROS_ERROR_STREAM_THROTTLE(2, "Maximum velocity exceeded. Right: "
                                         << vel.right_velocity << "(m/s), Left: " << vel.left_velocity
                                         << "(m/s), linear velocity: " << vehicle_velocity
                                         << "(m/s), Max: " << maximum_vel_ << "(m/s) ... Stopping robot...");
        vel.right_velocity = 0;
        vel.left_velocity = 0;
      }
      cmd_pub_.publish(vel);  // pub velocity command
    }
    rate.sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_follower");
  PathFollower path_follower;
  return 0;
}
