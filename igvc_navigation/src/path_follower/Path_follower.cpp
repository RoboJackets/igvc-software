#define _USE_MATH_DEFINES

#include <cmath>
#include <iostream>
#include <Eigen/Dense>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <igvc_msgs/velocity_pair.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_datatypes.h>

#include <igvc_utils/NodeUtils.hpp>
#include <igvc_utils/RobotState.hpp>

#include "Path_follower.h"


Path_follower::Path_follower() {
  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  // load controller parameters
  double target_velocity;
  double axle_length;
  double k1;
  double k2;
  double granularity;
  double lookahead_dist;
  igvc::param(pNh, "target_v", target_velocity, 1.0);
  igvc::param(pNh, "axle_length", axle_length, 0.52);
  igvc::param(pNh, "k1", k1, 1.0);
  igvc::param(pNh, "k2", k2, 3.0);
  igvc::param(pNh, "granularity", granularity, 2.0);
  igvc::param(pNh, "lookahead_dist", lookahead_dist, 2.0);
  controller = std::unique_ptr<Smooth_control>(new Smooth_control{k1, k2, axle_length, granularity, target_velocity, lookahead_dist});

  // load global parameters
  igvc::param(pNh, "maximum_vel", maximum_vel, 1.6);
  igvc::param(pNh, "stop_dist", stop_dist, 0.9);

  ros::Subscriber path_sub = nh.subscribe("/path", 1, &Path_follower::path_callback, this);
  ros::Subscriber pose_sub = nh.subscribe("/odometry/filtered", 1, &Path_follower::position_callback, this);
  ros::Subscriber waypoint_sub = nh.subscribe("/waypoint", 1, &Path_follower::waypoint_callback, this);

  cmd_pub = nh.advertise<igvc_msgs::velocity_pair>("/motors", 1);
  target_pub = nh.advertise<geometry_msgs::PointStamped>("/target_point", 1);
  trajectory_pub = nh.advertise<nav_msgs::Path>("/trajectory", 1);

  ros::spin();
}

void Path_follower::path_callback(const nav_msgs::PathConstPtr& msg)
{
  ROS_DEBUG_STREAM("Follower got path. Size: " << msg->poses.size());
  path = msg;
}

void Path_follower::waypoint_callback(const geometry_msgs::PointStampedConstPtr& msg)
{
  waypoint = msg;
}

/**
Constructs a new trajectory to follow using the current path msg and publishes
the first velocity command from this trajectory.
*/
void Path_follower::position_callback(const nav_msgs::OdometryConstPtr& msg)
{
  if (path.get() == nullptr)
  {
    return;
  }
  if (path->poses.empty() || path->poses.size() < 2)
  {
    ROS_INFO("Path empty.");
    igvc_msgs::velocity_pair vel;
    vel.left_velocity = 0.;
    vel.right_velocity = 0.;
    cmd_pub.publish(vel);
    path.reset();
    return;
  }

  // Current pose
  RobotState cur_pos(msg);

  double goal_dist = cur_pos.distTo(waypoint->point.x, waypoint->point.y);

  ROS_DEBUG_STREAM("Distance to waypoint: " << goal_dist << "(m.)");

  ros::Time time = msg->header.stamp;
  igvc_msgs::velocity_pair vel;  // immediate velocity command
  vel.header.stamp = time;

  if (goal_dist <= stop_dist)
  {
    /**
    Stop when the robot is a set distance away from the waypoint
    */
    ROS_INFO_STREAM(">>>WAYPOINT REACHED...STOPPING<<<");
    vel.right_velocity = 0;
    vel.left_velocity = 0;
    // make path null to stop planning until path generated
    // for new waypoint
    path = nullptr;
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
    controller->get_trajectory(vel, path, trajectory_msg, cur_pos, target);
    // publish trajectory
    trajectory_pub.publish(trajectory_msg);

    // publish target position
    geometry_msgs::PointStamped target_point;
    target_point.header.frame_id = "/odom";
    target_point.header.stamp = time;
    target_point.point.x = target.x;
    target_point.point.y = target.y;
    target_pub.publish(target_point);

    ROS_DEBUG_STREAM("Distance to target: " << cur_pos.distTo(target) << "(m.)");
  }

  // make sure maximum velocity not exceeded
  if (std::max(std::abs(vel.right_velocity), std::abs(vel.left_velocity)) > maximum_vel)
  {
    ROS_ERROR_STREAM_THROTTLE(5, "Maximum velocity exceeded. Right: " << vel.right_velocity << "(m/s), Left: " << vel.left_velocity
                                                          << "(m/s), Max: " << maximum_vel
                                                          << "(m/s) ... Stopping robot...");
    vel.right_velocity = 0;
    vel.left_velocity = 0;
  }

  cmd_pub.publish(vel);  // pub velocity command
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_follower");
  Path_follower path_follower;
  return 0;
}
