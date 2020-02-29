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
#include <gtsam/base/Vector.h>

#include <cmath>

#include <boost/shared_ptr.hpp>

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
   *  Use initial state to set FG priors and initialize values.
   */
  void initialize_pose_graph(void);

  /**
      Publish the estimated trajectory.
  */
  void publish_path();

  // ---------- Front End ----------- //

private:
  std::shared_ptr<gtsam::PreintegrationType> imu_preintegrated_ = nullptr;

  // Initial values to anchor FG.
  bool initial_imu_reading_received_ = false;  // Need initial rotation (R4) and vel (R3).
  bool initial_gps_coords_received = false;  // Need initial GPS coords (R3).
  // Format: (N,E,D,qX,qY,qZ,qW,velN,velE,velD)
  Eigen::Matrix<double,10,1> initial_state_ = Eigen::Matrix<double,10,1>::Zero();
  bool fg_initialized_ = false;  // Priors set and optimization has begun.

  // Solution initialization values.
  gtsam::Values initial_values_;

  int correction_count;  // Number of poses thus far.

  // The factor graph.
  gtsam::NonlinearFactorGraph *graph_ = new gtsam::NonlinearFactorGraph();

  // Assume zero initial bias.
  gtsam::imuBias::ConstantBias prior_imu_bias_; // assume zero initial bias

  // Used to convert from llh to ned.
  geodetic_converter::GeodeticConverter geodetic_converter_{};

  // Sensor specs.
  // TODO(alescontrela): Change these to match our actual IMU & GPS specs.
  double accel_noise_sigma_ = 0.000394;
  double gyro_noise_sigma_ = 0.000205;
  double accel_bias_rw_sigma_ = 0.004905;
  double gyro_bias_rw_sigma_ = 0.00000145;
  gtsam::Matrix33 measured_acc_cov_ =
    gtsam::Matrix33::Identity(3,3) * std::pow(accel_noise_sigma_, 2);
  gtsam::Matrix33 measured_omega_cov_ =
    gtsam::Matrix33::Identity(3,3) * std::pow(gyro_noise_sigma_, 2);
  gtsam::Matrix33 integration_error_cov_ =
    gtsam::Matrix33::Identity(3,3) * 1e-8;
  gtsam::Matrix33 bias_acc_cov_ =
    gtsam::Matrix33::Identity(3,3) * std::pow(accel_bias_rw_sigma_, 2);
  gtsam::Matrix33 bias_omega_cov_ =
    gtsam::Matrix33::Identity(3,3) * std::pow(gyro_bias_rw_sigma_, 2);
  gtsam::Matrix66 bias_acc_omega_int_ =
    gtsam::Matrix::Identity(6, 6) * 1e-5;
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> p_ =
    gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);

  // Assemble prior noise models.
  // Pose: rad, rad, rad, m, m, m.
  gtsam::noiseModel::Diagonal::shared_ptr pose_noise_model_ =
    gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished());
  // Velocity: m/s, m/s, m/s.
  gtsam::noiseModel::Diagonal::shared_ptr velocity_noise_model_ =
    gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  // Bias.
  gtsam::noiseModel::Diagonal::shared_ptr bias_noise_model_ =
    gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);

  // Number of measurements added to FG.
  int correction_count_ = 0;

  // Store previous state for the imu integration and the latest
  // predicted outcome.
  gtsam::NavState prev_state_;
  gtsam::NavState prop_state_;
  gtsam::imuBias::ConstantBias prev_bias_;

  double last_imu_msg_t_ = -1;

};

#endif
