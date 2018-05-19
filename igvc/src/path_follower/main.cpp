#define _USE_MATH_DEFINES

#include <igvc_msgs/velocity_pair.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <iostream>
#include <mutex>
#include <Eigen/Dense>

ros::Publisher cmd_pub;
ros::Publisher target_pub;

nav_msgs::PathConstPtr path;

double path_resolution;

double max_vel, axle_length, lookahead_dist, threshold;

void path_callback(const nav_msgs::PathConstPtr& msg)
{
  ROS_INFO("Follower got path");
  path = msg;
}

void get_wheel_speeds(igvc_msgs::velocity_pair& vel, double d, double theta)
{
  if(std::abs(theta) < threshold) {
    vel.right_velocity = max_vel;
    vel.left_velocity = max_vel;
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

double get_distance(double x1, double y1, double x2, double y2) {
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

void position_callback(const nav_msgs::OdometryConstPtr& msg)
{
  if (path.get() == nullptr)
  {
    return;
  }
  if (path->poses.empty())
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

  float tar_x, tar_y;
  geometry_msgs::Point end = path->poses[path->poses.size() - 1].pose.position;
  if(get_distance(cur_x, cur_y, end.x, end.y) > lookahead_dist) {
    ROS_INFO("got here");
    double path_index = 0;
    double distance = 0;
    bool cont = true;
    while(cont && path_index < path->poses.size() - 1) {
      geometry_msgs::Point point1 = path->poses[path_index].pose.position;
      geometry_msgs::Point point2 = path->poses[path_index + 1].pose.position;
      double increment = get_distance(point1.x, point1.y, point2.x, point2.y);
      if(distance + increment >lookahead_dist) {
        cont = false;
        Eigen::Vector3d first(point1.x,point1.y, 0);
        Eigen::Vector3d second(point2.x,point2.y, 0);
        Eigen::Vector3d slope = second - first;
        slope *= lookahead_dist;
        slope += first;
        tar_x = slope[0];
        tar_y = slope[1];
      } else {
        path_index++;
        distance += increment;
      }
    }
  } else {
    ROS_INFO("else");
    tar_x = end.x;
    tar_y = end.y;
  }

  geometry_msgs::PointStamped target_point;
  target_point.point.x = tar_x;
  target_point.point.y = tar_y;
  target_point.header.frame_id = "/odom";
  target_point.header.stamp = ros::Time::now();
  target_pub.publish(target_point);


  double d = std::sqrt(std::pow(tar_x - cur_x, 2) + std::pow(tar_y - cur_y, 2));
  double ang = std::atan2(tar_x - cur_x, tar_y - cur_y);
  ang = ang < 0 ? ang + 2 * M_PI : ang;
  double theta = cur_theta - ang;

  igvc_msgs::velocity_pair vel;
  get_wheel_speeds(vel, d, theta);

  ROS_INFO_STREAM(vel.left_velocity << ", " << vel.right_velocity);
  cmd_pub.publish(vel);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_follower");

  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  pNh.param(std::string("max_vel"), max_vel, 1.6);
  pNh.param(std::string("axle_length"), axle_length, 0.52);
  pNh.param(std::string("lookahead_dist"), lookahead_dist, 0.8);

  cmd_pub = nh.advertise<igvc_msgs::velocity_pair>("/motors", 1);
  target_pub = nh.advertise<geometry_msgs::PointStamped>("/target_point", 1);

  ros::Subscriber path_sub = nh.subscribe("/path_display", 1, path_callback);

  ros::Subscriber pose_sub = nh.subscribe("/odometry/filtered", 1, position_callback);


  ros::spin();

  return 0;
}
