#define _USE_MATH_DEFINES

#include <igvc_msgs/lights.h>
#include <igvc_msgs/velocity_pair.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>

const float threshold = 0.1;

ros::Publisher cmd_pub;

nav_msgs::PathConstPtr path;
std::mutex path_mutex;

size_t path_index = 0;

double max_vel, axle_length;

void path_callback(const nav_msgs::PathConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(path_mutex);
  ROS_INFO("Follower got path");
  path = msg;
  path_index = 0;
}

void get_wheel_speeds(igvc_msgs::velocity_pair& vel, double d, double theta)
{
  if (theta > M_PI + threshold)
  {
    theta -= 2 * M_PI;
  }
  else if (theta < -M_PI - threshold)
  {
    theta += 2 * M_PI;
  }

  if (std::abs(theta) < threshold)
  {
    vel.left_velocity = max_vel;
    vel.right_velocity = max_vel;
    return;
  }
  if (theta > M_PI / 4 || theta < -M_PI / 4)
  {
    if (theta > 0)
    {
      vel.left_velocity = -max_vel;
      vel.right_velocity = max_vel;
    }
    else
    {
      vel.left_velocity = max_vel;
      vel.right_velocity = -max_vel;
    }
    return;
  }

  double turn_radius = std::abs(d / (2 * std::sin(theta)));
  double outer_wheel = max_vel;
  double inner_wheel = outer_wheel * (turn_radius - (axle_length / 2)) / (turn_radius + (axle_length / 2));
  
  if (theta > 0)
  {
    vel.left_velocity = inner_wheel;
    vel.right_velocity = outer_wheel;
  }
  else
  {
    vel.left_velocity = outer_wheel;
    vel.right_velocity = inner_wheel;
  }
}

void position_callback(const nav_msgs::OdometryConstPtr& msg)
{
  if (path.get() == nullptr)
  {
    return;
  }
  if (path->poses.empty() || path_index == path->poses.size())
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
  float cur_theta = -tf::getYaw(q) + M_PI / 2;
  cur_theta = cur_theta > 2 * M_PI ? cur_theta - 2 * M_PI : cur_theta;
    
  float tar_x = path->poses[path_index].pose.position.x;
  float tar_y = path->poses[path_index].pose.position.y;

  while (std::abs(cur_x - tar_x) + std::abs(cur_y - tar_y) < threshold)
  {
    ++path_index;
    if (path_index == path->poses.size())
    {
      // relying on the next odom update to stop the robot
      // if odom is slow, this may overshoot the target
      return;
    }
    tar_x = path->poses[path_index].pose.position.x;
    tar_y = path->poses[path_index].pose.position.y;
  }

  double d = std::sqrt(std::pow(tar_x - cur_x, 2) + std::pow(tar_y - cur_y, 2));
  double ang = std::atan2(tar_x - cur_x, tar_y - cur_y);
  ang = ang < 0 ? ang + 2 * M_PI : ang;
  double theta = cur_theta - ang;

  igvc_msgs::velocity_pair vel;
  get_wheel_speeds(vel, d, theta);

  cmd_pub.publish(vel);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_follower");

  ros::NodeHandle n;

  n.param(std::string("max_vel"), max_vel, 1.6);
  n.param(std::string("axle_length"), axle_length, 0.8);

  cmd_pub = n.advertise<igvc_msgs::velocity_pair>("/motors", 1);

  ros::Publisher lights_pub = n.advertise<igvc_msgs::lights>("/lights", 1);

  ros::Subscriber path_sub = n.subscribe("/path_display", 1, path_callback);

  ros::Subscriber pose_sub = n.subscribe("/odometry/filtered", 1, position_callback);

  igvc_msgs::lights lights_cmd;
  lights_cmd.safety_flashing = true;
  lights_pub.publish(lights_cmd);

  ros::spin();

  return 0;
}
