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

double k1, k2;

void path_callback(const nav_msgs::PathConstPtr& msg)
{
  ROS_INFO("Follower got path");
  path = msg;
}

void get_wheel_speeds(igvc_msgs::velocity_pair& vel, double d, double theta, double delta)
{
  ROS_INFO_STREAM("d = " << d << " theta = " << theta << " delta = " << delta);
  double K = k2 * (delta = atan(-k1*theta));
  K += (1 + (k1/(1 + pow(k1 * theta, 2))))*sin(delta);
  K /= -d;
  double w = K * max_vel;

  vel.left_velocity = max_vel - w * axle_length / 2;
  vel.right_velocity = max_vel + w * axle_length / 2;
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
  float cur_theta = tf::getYaw(q);

  float tar_x, tar_y;
  geometry_msgs::Point end = path->poses[path->poses.size() - 1].pose.position;
  double theta;
  double delta;
  if(get_distance(cur_x, cur_y, end.x, end.y) > lookahead_dist && path->poses.size() >= 2) {
    double path_index = 0;
    double distance = 0;
    bool cont = true;
    while(cont && path_index < path->poses.size() - 1) {
      geometry_msgs::Point point1, point2;
      if(path->poses.size() == 1) {
        point1 = msg->pose.pose.position;
        point2 = path->poses[0].pose.position;
      } else {
        point1 = path->poses[path_index].pose.position;
        point2 = path->poses[path_index + 1].pose.position;
      }
      double increment = get_distance(point1.x, point1.y, point2.x, point2.y);
      if(distance + increment > lookahead_dist) {
        cont = false;
        Eigen::Vector3d first(point1.x,point1.y, 0);
        Eigen::Vector3d second(point2.x,point2.y, 0);
        Eigen::Vector3d slope = second - first;
        if(path_index + 2 > path->poses.size()) {
          geometry_msgs::Point point3 = path->poses[path_index + 2].pose.position;
          theta = -atan((point3.y - point2.y)/(point3.x - point2.x));
          delta = -atan(slope[1]/slope[0]);
        } else {
          theta = -atan(slope[1]/slope[0]);
          delta = theta;
        }
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
    tar_x = end.x;
    tar_y = end.y;
    theta = cur_theta;
    delta = cur_theta;
  }

  ros::Time time = ros::Time::now();

  geometry_msgs::PointStamped target_point;
  target_point.point.x = tar_x;
  target_point.point.y = tar_y;
  target_point.header.frame_id = "/odom";
  target_point.header.stamp = time;
  target_pub.publish(target_point);
  ROS_INFO_STREAM("target = " << tar_x << ", " << tar_y);

  double d = get_distance(cur_x, cur_y, tar_x, tar_y);

  igvc_msgs::velocity_pair vel;
  get_wheel_speeds(vel, d, theta, delta);
  vel.header.stamp = time;

  cmd_pub.publish(vel);
  ROS_INFO_STREAM("PUB " << vel.right_velocity << " " << vel.left_velocity);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_follower");

  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  pNh.param(std::string("max_vel"), max_vel, 1.6);
  pNh.param(std::string("axle_length"), axle_length, 0.52);
  pNh.param(std::string("lookahead_dist"), lookahead_dist, 0.8);
  pNh.param(std::string("k1"), k1, 1.0);
  pNh.param(std::string("k2"), k2, 3.0);

  ros::Subscriber path_sub = nh.subscribe("/path", 1, path_callback);

  ros::Subscriber pose_sub = nh.subscribe("/odometry/filtered", 1, position_callback);

  cmd_pub = nh.advertise<igvc_msgs::velocity_pair>("/motors", 1);

  target_pub = nh.advertise<geometry_msgs::PointStamped>("/target_point", 1);

  ros::spin();

  return 0;
}
