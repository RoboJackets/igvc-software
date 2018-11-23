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
  ROS_INFO_STREAM("Follower got path. Size: " << msg->poses.size());
  path = msg;
}

double get_distance(double x1, double y1, double x2, double y2)
{
  /**
  Calculates euclidian distance between two points
  */
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

void computeAngle(float& angle, Eigen::Vector3d vec2, Eigen::Vector3d vec1)
{
  /**
  Computes the egocentric polar angle of vec2 wrt vec1 in 2D, that is:
    - clockwise: negative
    - counter-clockwise: positive

  source: https://stackoverflow.com/questions/14066933/direct-way-of-computing-clockwise-angle-between-2-vectors
  */

  double dot = vec2[0]*vec1[0] + vec2[1]*vec1[1]; // dot product - proportional to cos
  double det = vec2[0]*vec1[1] - vec2[1]*vec1[0]; // determinant - proportional to sin

  angle = atan2(det, dot);
}

void position_callback(const nav_msgs::OdometryConstPtr& msg)
{
  /**
  Constructs a new trajectory to follow using the current path msg and publishes
  the first velocity command from this trajectory.
  */
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
  float cur_x = msg->pose.pose.position.x;
  float cur_y = msg->pose.pose.position.y;
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
  float orientation = tf::getYaw(q);

  // target position
  float tar_x, tar_y;

  /**
  Find the index of the closest point on the path.
  */
  geometry_msgs::Point end = path->poses[path->poses.size() - 1].pose.position;
  double path_index = 0;
  double closest = get_distance(
                       cur_x,
                       cur_y,
                       path->poses[0].pose.position.x,
                       path->poses[0].pose.position.y
                    );

  double temp = get_distance(
                    cur_x,
                    cur_y,
                    path->poses[path_index].pose.position.x,
                    path->poses[path_index].pose.position.y
                );

  while (path_index < path->poses.size() && temp <= closest)
  {
    if (temp < closest)
    {
      closest = temp;
    }
    path_index++;
    temp = get_distance(
               cur_x,
               cur_y,
               path->poses[path_index].pose.position.x,
               path->poses[path_index].pose.position.y
           );
  }

  /**
  Find the furthest point along trajectory that isn't further than the
  lookahead distance. This is the target position.
  */
  if (get_distance(cur_x, cur_y, end.x, end.y) > lookahead_dist)
  {
    double distance = 0; // Eigen::Vector3d init_orientation = los - heading;
  // cur_theta = orientation - std::atan2(init_orientation[1], init_orientation[0]);
    bool cont = true;

    while (cont && path_index < path->poses.size() - 1)
    {
      geometry_msgs::Point point1, point2;
      point1 = path->poses[path_index].pose.position;
      point2 = path->poses[path_index + 1].pose.position;
      double increment = get_distance(
                            point1.x,
                            point1.y,
                            point2.x,
                            point2.y
                        );

      if (distance + increment > lookahead_dist)
      {
        cont = false;
        Eigen::Vector3d first(point1.x, point1.y, 0);
        Eigen::Vector3d second(point2.x, point2.y, 0);
        Eigen::Vector3d slope = second - first;

        slope /= increment;
        slope *= (distance - lookahead_dist) + increment;

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

  /**
  Calculate the line of sight (los) from the robot to the target position in
  vector format.
  */
  double slope_x = tar_x - cur_x;
  double slope_y = tar_y - cur_y;

  Eigen::Vector3d los(slope_x, slope_y, 0); // line of sight
  los.normalize();

  /**
  Calculate cur_theta, the angle between the los and the current robot heading,
  adjusted for the robot's current orientation in space.
  */
  // get current robot heading in vector format
  Eigen::Vector3d heading(std::cos(orientation), std::sin(orientation), 0);

  float cur_theta;
  computeAngle(cur_theta, los, heading);
  // cur_theta = orientation - cur_theta;

  // Eigen::Vector3d init_orientation = los - heading;
  // cur_theta = orientation - std::atan2(init_orientation[1], init_orientation[0]);


  /**
  Calculate target theta (tar_theta), the angle between the los and the target
  pose, adjusted for the robot's current orientation in space.
  */
  // get i and j components of target orientation vector (res_orientation)
  double distance = 0;
  geometry_msgs::Point point1, point2;
  unsigned int path_idx = 0;
  while (path_idx < path->poses.size() - 1)
  {
    point1 = path->poses[path_idx].pose.position;
    point2 = path->poses[path_idx + 1].pose.position;
    double increment = get_distance(
                            point1.x,
                            point1.y,
                            point2.x,
                            point2.y
                        );

    if (distance + increment > lookahead_dist) { break; }

    path_idx++;
    distance += increment;
  }

  double pose_x = point2.x - point1.x;
  double pose_y = point2.y - point1.y;

  Eigen::Vector3d tar_orientation(pose_x, pose_y, 0); // target orientation
  tar_orientation.normalize();

  float tar_theta;
  computeAngle(tar_theta, los, tar_orientation);
  // tar_theta = cur_theta + tar_theta

  // Eigen::Vector3d delta_orientation = tar_orientation - los;
  // tar_theta = orientation - atan2(delta_orientation[1], delta_orientation[0]);

  ROS_INFO_STREAM("Orientation:" << orientation << " DELTA: " << cur_theta << " THETA: " << tar_theta);

  ros::Time time = ros::Time::now();

  // publish target position
  geometry_msgs::PointStamped target_point;
  target_point.header.frame_id = "/odom";
  target_point.header.stamp = time;
  target_point.point.x = tar_x;
  target_point.point.y = tar_y;
  target_pub.publish(target_point);

  /**
  Obtain smooth control law from the controller. This includes a smooth
  trajectory for visualization purposes and an immediate velocity command.
  */
  igvc_msgs::velocity_pair vel;
  vel.header.stamp = time;

  nav_msgs::Path trajectory_msg;
  trajectory_msg.header.stamp = time;
  trajectory_msg.header.frame_id = "/odom";

  Eigen::Vector3d cur_pos(cur_x, cur_y, cur_theta);
  Eigen::Vector3d target(tar_x, tar_y, tar_theta);
  controller.getTrajectory(vel, trajectory_msg, cur_pos, target);

  ROS_INFO_STREAM("Distance: "
                  << get_distance(tar_x, tar_y, cur_x, cur_y)
                  << "m.");

  // make sure maximum velocity not exceeded
  if (std::max(vel.right_velocity, vel.left_velocity) > maximum_vel)
  {
    ROS_ERROR_STREAM("Maximum velocity exceeded. Right: "
                            << vel.right_velocity
                            << ", Left: "
                            << vel.left_velocity
                            << ", Max: "
                            << maximum_vel
                            << "Stopping robot...");
    vel.right_velocity = 0;
    vel.left_velocity = 0;
  }

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
