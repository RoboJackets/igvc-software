#include "Odometer.h"
#include <igvc_msgs/velocity_pair.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include "Odometer.h"

double wheel_separation;

/**
 * Coneverts wheel velocities to odometry message using trigonometry for calculations
 * In the ros coordinate convention x is forward, y is leftward, and z is upward relative to the robot
 * The position is published in an absolute reference frame relative to the initial position
 * The velocities (twist) is in a reference frame relative to the robot
 */
void Odometer::enc_callback(const igvc_msgs::velocity_pair& msg)
{
  float leftVelocity = msg.left_velocity;
  float rightVelocity = msg.right_velocity;
  float deltaT = msg.duration;

  float angularVelocity = (rightVelocity - leftVelocity) / wheel_sep;
  float deltaTheta = angularVelocity * deltaT;
  float velocity = (rightVelocity + leftVelocity) / 2;

  geometry_msgs::Vector3 linearVelocities;
  linearVelocities.z = 0;

  if (fabs(rightVelocity - leftVelocity) > 1e-4)
  {  // 1e-4 is the point where less of a difference is straight
    linearVelocities.y = velocity * sin(deltaTheta);
    linearVelocities.x = velocity * cos(deltaTheta);
  }
  else
  {
    // limit where turn radius is infinite (ie. straight line)
    linearVelocities.y = 0;
    linearVelocities.x = velocity;
  }

  // set angular velocities - assuming 2D operation
  geometry_msgs::Vector3 angularVelocities;
  angularVelocities.x = 0.0;
  angularVelocities.y = 0.0;
  angularVelocities.z = angularVelocity;

  nav_msgs::Odometry odom;
  odom.twist.twist.linear = linearVelocities;
  odom.twist.twist.angular = angularVelocities;

  // update global positions
  // note x and y velocities are local reference frame - convert to global then increment poition
  float dx = linearVelocities.x * deltaT;
  float dy = linearVelocities.y * deltaT;
  x += dx * cos(yaw) - dy * sin(yaw);
  y += dx * sin(yaw) + dy * cos(yaw);
  yaw += deltaTheta;

  // enter message info for global position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

  // Row-major representation of the 6x6 covariance matrix
  // The orientation parameters use a fixed-axis representation.
  // In order, the parameters are:
  // (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
  odom.twist.covariance = { 0.02, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 0.25, 1e-4, 1e-4, 1e-4, 1e-4,
                            1e-4, 1e-4, 1e6,  1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e6,  1e-4, 1e-4,
                            1e-4, 1e-4, 1e-4, 1e-4, 1e6,  1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 0.62 };
  // the position covariance takes same form as twist covariance above
  // this grows without bounds as error accumulates - disregard exact reading with high variance
  odom.pose.covariance = { 0.01, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 0.01, 1e-6, 1e-6, 1e-6, 1e-6,
                           1e-6, 1e-6, 1e6,  1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e6,  1e-6, 1e-6,
                           1e-6, 1e-6, 1e-6, 1e-6, 1e6,  1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e6 };

  // setting sequence of message
  odom.header.seq = seq++;

  // setting reference frames
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";

  // set time then publish
  odom.header.stamp = ros::Time::now();
  pub.publish(odom);
}

Odometer::Odometer(ros::NodeHandle& nh)
{
  nh.param("wheel_sep", wheel_sep, 0.52);

  sub = nh.subscribe("/encoders", 10, &Odometer::enc_callback, this);
  pub = nh.advertise<nav_msgs::Odometry>("/wheel_odometry", 10);

  // initializing sequence number for messages
  seq = 0;

  // initialize position - map published is relative to position at time t=0
  x = 0;
  y = 0;
  yaw = 0;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wheel_odom");
  ros::NodeHandle nh;

  ros::NodeHandle pNh("~");

  pNh.param("wheel_separation", wheel_separation, 0.83);

  Odometer odom(nh);

  ROS_INFO_STREAM("wheel odometry node has started");

  ros::spin();
}
