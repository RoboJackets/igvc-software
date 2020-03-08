/**
Pose SLAM Implementation

@file PoseSlamEstimator.h
@brief PoseSLAM with GTSAM on ROS.
@author: Alejandro Escontrela <aescontrela3@gatech.edu>
Date Created: January 22nd, 2020
*/

#ifndef POSESLAMESTIMATOR_H
#define POSESLAMESTIMATOR_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>

#include <cmath>
#include <boost/shared_ptr.hpp>

#include "geodetic_conv.hpp"
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

class PoseSlamEstimator
{
public:
  PoseSlamEstimator(ros::NodeHandle* nodehandle);

  /**
      Get the most recent IMU reading and integrate it.
      @param[in] msg the imu reading.
  */
  void imu_callback(const sensor_msgs::ImuConstPtr& msg);

  /**
      Get the most recent gps reading and optimize the factor graph for poses.
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
  void publish_path(void);

private:
  // Node handler, publishers and subscribers.
  ros::NodeHandle nh_;
  ros::Subscriber imu_sub_, gps_sub_;
  ros::Publisher trajectory_publisher_;

  // Params.
  double rate_;
  double accel_noise_sigma_;
  double gyro_noise_sigma_;
  double accel_bias_rw_sigma_;
  double gyro_bias_rw_sigma_;
  std::vector<double> pose_noise_elements_;  // rad, rad, rad, m, m, m.
  double velocity_noise_elements_;           // m/s.
  double bias_noise_elements_;
  double gps_noise_elements_;

  // Integrates IMU measurements and covariance matrices.
  std::shared_ptr<gtsam::PreintegrationType> imu_preintegrated_ = nullptr;

  // Initial values to anchor FG. Format: (N,E,D,qX,qY,qZ,qW,velN,velE,velD)
  bool fg_initialized_ = false;                // Priors set and optimization has begun.
  bool initial_imu_reading_received_ = false;  // Initial rotation and vel.
  bool initial_gps_coords_received_ = false;    // Initial GPS coords.
  Eigen::Matrix<double, 10, 1> initial_state_ = Eigen::Matrix<double, 10, 1>::Zero();

  // Solution initialization values and optimization results.
  gtsam::Values initial_values_, results_;

  // The factor graph.
  gtsam::NonlinearFactorGraph* graph_ = new gtsam::NonlinearFactorGraph();

  // Assume zero initial bias.
  gtsam::imuBias::ConstantBias prior_imu_bias_;

  // Used to convert from llh to ned.
  // const GeographicLib::Geocentric& earth_ = GeographicLib::Geocentric::WGS84();
  GeographicLib::LocalCartesian enu_;
  geodetic_converter::GeodeticConverter geodetic_converter_{};

  // Measured sensor variances and biases.
  gtsam::Matrix33 measured_acc_cov_, measured_omega_cov_, integration_error_cov_, bias_acc_cov_, bias_omega_cov_;
  gtsam::Matrix66 bias_acc_omega_int_;
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> p_ =
      gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(9.8);

  // Assemble prior noise models.
  gtsam::noiseModel::Diagonal::shared_ptr pose_noise_model_;
  gtsam::noiseModel::Diagonal::shared_ptr velocity_noise_model_;
  gtsam::noiseModel::Diagonal::shared_ptr bias_noise_model_;

  // Number of measurements added to FG.
  int correction_count_ = 0;

  // Store previous state for the imu integration and the latest predicted outcome.
  gtsam::NavState prev_state_;
  gtsam::NavState prop_state_;
  gtsam::imuBias::ConstantBias prev_bias_;

  // UTM to odom transform.
  gtsam::Pose3 oTutm_;

  // For debug. TODO(alescontrela): remove.
  gtsam::Quaternion current_orientation_;

  double last_imu_msg_t_;
};

#endif
