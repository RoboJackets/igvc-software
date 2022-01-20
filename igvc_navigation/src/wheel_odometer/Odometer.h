#ifndef ODOMETER_H
#define ODOMETER_H
#include <igvc_msgs/velocity_pair.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <math.h>
#include <nav_msgs/Odometry.h>

class Odometer
{
public:
  Odometer(ros::NodeHandle&);

private:
  // ros infastructure
  ros::Publisher pub;
  ros::Subscriber sub;
  tf::TransformBroadcaster odom_broadcaster;
  int seq;

  // robot constant
  double wheel_sep;

  // keeping track of global position
  float x;
  float y;
  float yaw;

  // callback for encoder subscriber
  void enc_callback(const igvc_msgs::velocity_pair&);
};
#endif
