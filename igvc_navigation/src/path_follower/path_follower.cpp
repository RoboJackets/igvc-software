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

#include "path_follower.h"

using igvc::Assertion;

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
  seconds simulation_horizon;
  igvc::param(pNh, "target_v", target_velocity, 1.0, Assertion::POSITIVE);
  igvc::param(pNh, "axle_length", axle_length, 0.52, Assertion::POSITIVE);
  igvc::param(pNh, "k1", k1, 1.0);
  igvc::param(pNh, "k2", k2, 3.0);
  igvc::param(pNh, "simulation_frequency", simulation_frequency, 100.0, Assertion::POSITIVE);
  igvc::param(pNh, "lookahead_dist", lookahead_dist, 2.0, Assertion::POSITIVE);
  igvc::param(pNh, "simulation_horizon", simulation_horizon, 5.0, Assertion::POSITIVE);

  controller_ = std::unique_ptr<SmoothControl>(new SmoothControl{
      k1, k2, axle_length, simulation_frequency, target_velocity, lookahead_dist, simulation_horizon });

  // load global parameters
  igvc::getParam(pNh, "maximum_vel", maximum_vel_, Assertion::POSITIVE);
  igvc::param(pNh, "stop_dist", stop_dist_, 0.9, Assertion::POSITIVE);

  ros::Subscriber path_sub = nh.subscribe("/path", 1, &PathFollower::pathCallback, this);
  ros::Subscriber pose_sub = nh.subscribe("/odometry/filtered", 1, &PathFollower::positionCallback, this);
  ros::Subscriber waypoint_sub = nh.subscribe("/waypoint", 1, &PathFollower::waypointCallback, this);

  cmd_pub_ = nh.advertise<igvc_msgs::velocity_pair>("/motors", 1);
  target_pub_ = nh.advertise<geometry_msgs::PointStamped>("/target_point", 1);
  trajectory_pub_ = nh.advertise<nav_msgs::Path>("/trajectory", 1);

  ros::spin();
}

void PathFollower::pathCallback(const nav_msgs::PathConstPtr& msg)
{
  ROS_DEBUG_STREAM("Follower got path. Size: " << msg->poses.size());
  //  path_ = msg;
  // TODO: Remove patch when motion planning correctly incorporates heading
  path_ = getPatchedPath(msg);
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
  if (path_.get() == nullptr || path_->poses.empty() || path_->poses.size() < 2)
  {
    ROS_INFO_THROTTLE(1, "Path empty.");
    igvc_msgs::velocity_pair vel;
    vel.left_velocity = 0.;
    vel.right_velocity = 0.;
    cmd_pub_.publish(vel);
    path_.reset();
    return;
  }

  // Current pose
  RobotState cur_pos(msg);

  double goal_dist = cur_pos.distTo(waypoint_->point.x, waypoint_->point.y);

  ROS_DEBUG_STREAM("Distance to waypoint: " << goal_dist << "(m.)");

  ros::Time time = msg->header.stamp;
  igvc_msgs::velocity_pair vel;  // immediate velocity command
  vel.header.stamp = time;

  if (goal_dist <= stop_dist_)
  {
    /**
    Stop when the robot is a set distance away from the waypoint
    */
    ROS_INFO_STREAM(">>>WAYPOINT REACHED...STOPPING<<<");
    vel.right_velocity = 0;
    vel.left_velocity = 0;
    // make path null to stop planning until path generated
    // for new waypoint
    path_ = nullptr;
  }
  else
  {
    /**
    Obtain smooth control law from the controller. This includes a smooth
    trajectory for visualization purposes and an immediate velocity command.
    */
    nav_msgs::Path trajectory_msg;
    trajectory_msg.header.stamp = time;
    trajectory_msg.header.frame_id = "/odom";

    RobotState target;
    controller_->getTrajectory(vel, path_, trajectory_msg, cur_pos, target);
    // publish trajectory
    trajectory_pub_.publish(trajectory_msg);

    // publish target position
    geometry_msgs::PointStamped target_point;
    target_point.header.frame_id = "/odom";
    target_point.header.stamp = time;
    target_point.point.x = target.x;
    target_point.point.y = target.y;
    target_pub_.publish(target_point);

    ROS_DEBUG_STREAM("Distance to target: " << cur_pos.distTo(target) << "(m.)");
  }

  // make sure maximum velocity not exceeded
  if (std::max(std::abs(vel.right_velocity), std::abs(vel.left_velocity)) > maximum_vel_)
  {
    ROS_ERROR_STREAM_THROTTLE(5, "Maximum velocity exceeded. Right: "
                                     << vel.right_velocity << "(m/s), Left: " << vel.left_velocity
                                     << "(m/s), Max: " << maximum_vel_ << "(m/s) ... Stopping robot...");
    vel.right_velocity = 0;
    vel.left_velocity = 0;
  }

  cmd_pub_.publish(vel);  // pub velocity command
}

nav_msgs::PathConstPtr PathFollower::getPatchedPath(const nav_msgs::PathConstPtr& msg) const
{
  nav_msgs::PathPtr new_path = boost::make_shared<nav_msgs::Path>(*msg);
  size_t num_poses = new_path->poses.size();
  for (int i = 0; i < num_poses; i++)
  {
    double delta_x = new_path->poses[i + 1].pose.position.x - new_path->poses[i].pose.position.x;
    double delta_y = new_path->poses[i + 1].pose.position.y - new_path->poses[i].pose.position.y;
    double heading = atan2(delta_y, delta_x);
    new_path->poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(heading);
  }
  new_path->poses.back().pose.orientation = new_path->poses[num_poses - 2].pose.orientation;
  return new_path;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_follower");
  PathFollower path_follower;
  return 0;
}
