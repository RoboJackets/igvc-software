/**
Pose SLAM Implementation

Author: Alejandro Escontrela <aescontrela3@gatech.edu>
Date Created: January 22nd, 2020
*/

#ifndef POSESLAMESTIMATOR_H
#define POSESLAMESTIMATOR_H



// ROS stuff.
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

// GTSAM stuff.
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>

#include "geodetic_conv.hpp"

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

class PoseSlamEstimator
{
public:
  // ---------- Node functionality ----------- //
  PoseSlamEstimator(ros::NodeHandle* nodehandle);
  ros::NodeHandle nh_;

  // subscribers
  ros::Subscriber imu_sub_;
  ros::Subscriber gps_sub_;
  // publishers
  ros::Publisher trajectory_publisher_;

  // launch parameters

  /**
      Get the most recent IMU reading
      @param[in] msg the imu reading.
  */
  void imu_callback(const sensor_msgs::ImuConstPtr& msg);

  /**
      Get the most recent gps reading.
      @param[in] msg the gps reading.
  */
  void gps_callback(const sensor_msgs::NavSatFixConstPtr& msg);

  /**
      Publish the estimated trajectory.
  */
  void publish_path();

  // ---------- Front End ----------- //

private:
  gtsam::PreintegrationType *imu_preintegrated_;

  // Need to obtain initial values to specify prior.
  bool initial_vel_received_;
  bool initial_orientation_received_;
  bool initial_gps_coords_received;

  gtsam::imuBias::ConstantBias prior_imu_bias_; // assume zero initial bias

  // Use whatever this is to convert llh to ned.
  // http://docs.ros.org/fuerte/api/enu/html/group__coord__system.html

};

#endif
