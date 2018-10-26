#define _USE_MATH_DEFINES

#include <igvc_msgs/velocity_pair.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include "SmoothControl.h"

ros::Publisher cmd_pub;
ros::Publisher target_pub;
ros::Publisher trajectory_pub;

nav_msgs::PathConstPtr path;

double lookahead_dist, maximum_vel;

SmoothControl controller;

void path_callback(const nav_msgs::PathConstPtr& msg)
{
  ROS_INFO("Follower got path");
  path = msg;
}

double get_distance(double x1, double y1, double x2, double y2)
{
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

void position_callback(const nav_msgs::OdometryConstPtr& msg)
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

  float cur_x = msg->pose.pose.position.x;
  float cur_y = msg->pose.pose.position.y;
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
  float cur_theta = tf::getYaw(q);

  float tar_x, tar_y, tar_theta;
  geometry_msgs::Point end = path->poses[path->poses.size() - 1].pose.position;
  double path_index = 0;
  double closest = std::abs(get_distance(cur_x, cur_y, path->poses[0].pose.position.x, path->poses[0].pose.position.y));
  double temp = std::abs(
      get_distance(cur_x, cur_y, path->poses[path_index].pose.position.x, path->poses[path_index].pose.position.y));
  while (path_index < path->poses.size() && temp <= closest)
  {
    if (temp < closest)
    {
      closest = temp;
    }
    path_index++;
    temp = std::abs(
        get_distance(cur_x, cur_y, path->poses[path_index].pose.position.x, path->poses[path_index].pose.position.y));
  }

  if (get_distance(cur_x, cur_y, end.x, end.y) > lookahead_dist)
  {
    double distance = 0;
    bool cont = true;
    while (cont && path_index < path->poses.size() - 1)
    {
      geometry_msgs::Point point1, point2;
      point1 = path->poses[path_index].pose.position;
      point2 = path->poses[path_index + 1].pose.position;
      double increment = get_distance(point1.x, point1.y, point2.x, point2.y);
      if (distance + increment > lookahead_dist)
      {
        cont = false;
        Eigen::Vector3d first(point1.x, point1.y, 0);
        Eigen::Vector3d second(point2.x, point2.y, 0);
        Eigen::Vector3d slope = second - first;
        // ROS_INFO_STREAM("first = " << first[0] << ", " << first[1]);
        // ROS_INFO_STREAM("slope = " << slope[0] << ", " << slope[1]);
        // ROS_INFO_STREAM("look = " << lookahead_dist << " dista = " << distance);
        // ROS_INFO_STREAM("increment = " << increment << " look - dist = " << (distance - lookahead_dist) + increment);
        slope /= increment;
        slope *= (distance - lookahead_dist) + increment;
        // ROS_INFO_STREAM("slope2 = " << slope[0] << ", " << slope[1]);
        slope += first;
        tar_x = slope[0];
        tar_y = slope[1];
      }
      else
      {
        path_index++;
        distance += increment;
      }
    }
  }
  else
  {
    tar_x = end.x;
    tar_y = end.y;
  }

  double yDiff = tar_y - cur_y;
  double xDiff = tar_x - cur_x;

  if (xDiff == 0)
  {
    tar_theta = yDiff > 0 ? M_PI : -M_PI;
  }
  else
  {
    tar_theta = atan2((yDiff), (xDiff));
  }

  ros::Time time = ros::Time::now();

  geometry_msgs::PointStamped target_point;
  target_point.header.frame_id = "/odom";
  target_point.header.stamp = time;
  target_point.point.x = tar_x;
  target_point.point.y = tar_y;
  target_pub.publish(target_point);

  igvc_msgs::velocity_pair vel;
  vel.header.stamp = time;

  nav_msgs::Path trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();
  trajectory_msg.header.frame_id = "/odom";

  Eigen::Vector3d cur_pos(cur_x, cur_y, cur_theta);
  Eigen::Vector3d target(tar_x, tar_y, tar_theta);
  controller.getTrajectory(vel, trajectory_msg, cur_pos, target);

  ROS_INFO_STREAM("distance = " << get_distance(tar_x, tar_y, cur_x, cur_y));

  if (vel.right_velocity > maximum_vel || vel.left_velocity > maximum_vel)
  {
    ROS_ERROR_STREAM("Large velocity output stopping " << vel.right_velocity << ", " << vel.left_velocity);
    vel.right_velocity = 0;
    vel.left_velocity = 0;
  }
  // ROS_INFO_STREAM("target " << tar_x << " " << tar_y << "\n");

  cmd_pub.publish(vel);
  trajectory_pub.publish(trajectory_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_follower");

  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  pNh.param(std::string("target_v"), controller.v, 1.0);
  pNh.param(std::string("axle_length"), controller.axle_length, 0.52);
  pNh.param(std::string("k1"), controller.k1, 1.0);
  pNh.param(std::string("k2"), controller.k2, 3.0);
  pNh.param(std::string("roll_out_time"), controller.rollOutTime, 2.0);
  pNh.param(std::string("lookahead_dist"), lookahead_dist, 2.0);
  pNh.param(std::string("maximum_vel"), maximum_vel, 1.6);

  ros::Subscriber path_sub = nh.subscribe("/path", 1, path_callback);

  ros::Subscriber pose_sub = nh.subscribe("/odometry/filtered", 1, position_callback);

  cmd_pub = nh.advertise<igvc_msgs::velocity_pair>("/motors", 1);

  target_pub = nh.advertise<geometry_msgs::PointStamped>("/target_point", 1);

  trajectory_pub = nh.advertise<nav_msgs::Path>("/trajectory", 1);

  ros::spin();

  return 0;
}
