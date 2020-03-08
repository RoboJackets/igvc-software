#include "PoseSlamEstimator.h"

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>



#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/base/Vector.h>

#include <parameter_assertions/assertions.h>
#include <igvc_utils/NodeUtils.hpp>

PoseSlamEstimator::PoseSlamEstimator(ros::NodeHandle* nodehandle) : nh_(*nodehandle)
{
  ros::NodeHandle pNh("~");

  // Subscribe to sensor readings.
  imu_sub_ = nh_.subscribe("/imu", 1, &PoseSlamEstimator::imu_callback, this);
  gps_sub_ = nh_.subscribe("/gps/filtered", 1, &PoseSlamEstimator::gps_callback, this);

  // Publish estimated trajectory.
  trajectory_publisher_ = nh_.advertise<nav_msgs::Path>("/pose_slam_trajectory", 1);

  assertions::getParam(pNh, "rate", rate_);
  assertions::getParam(pNh, "accel_noise_sigma", accel_noise_sigma_);
  assertions::getParam(pNh, "gyro_noise_sigma", gyro_noise_sigma_);
  assertions::getParam(pNh, "accel_bias_rw_sigma", accel_bias_rw_sigma_);
  assertions::getParam(pNh, "gyro_bias_rw_sigma", gyro_bias_rw_sigma_);
  assertions::getParam(pNh, "pose_noise_model", pose_noise_elements_);
  assertions::getParam(pNh, "velocity_noise_model", velocity_noise_elements_);
  assertions::getParam(pNh, "bias_noise_model", bias_noise_elements_);
  assertions::getParam(pNh, "gps_noise_model", gps_noise_elements_);

  pose_noise_model_ = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << pose_noise_elements_[0], pose_noise_elements_[1], pose_noise_elements_[2],
       pose_noise_elements_[3], pose_noise_elements_[4], pose_noise_elements_[5])
          .finished());
  velocity_noise_model_ = gtsam::noiseModel::Isotropic::Sigma(3, velocity_noise_elements_);
  bias_noise_model_ = gtsam::noiseModel::Isotropic::Sigma(6, bias_noise_elements_);

  measured_acc_cov_ = gtsam::Matrix33::Identity(3, 3) * std::pow(accel_noise_sigma_, 2);
  measured_omega_cov_ = gtsam::Matrix33::Identity(3, 3) * std::pow(gyro_noise_sigma_, 2);
  integration_error_cov_ = gtsam::Matrix33::Identity(3, 3) * 1e-8;
  bias_acc_cov_ = gtsam::Matrix33::Identity(3, 3) * std::pow(accel_bias_rw_sigma_, 2);
  bias_omega_cov_ = gtsam::Matrix33::Identity(3, 3) * std::pow(gyro_bias_rw_sigma_, 2);
  bias_acc_omega_int_ = gtsam::Matrix::Identity(6, 6) * 1e-5;

  p_->setAccelerometerCovariance(measured_acc_cov_);     // Acc white noise in continuous.
  p_->setIntegrationCovariance(integration_error_cov_);  // Integration uncertainty continuous.
  p_->setGyroscopeCovariance(measured_omega_cov_);       // Gyro white noise in continuous.
  p_->setBiasAccCovariance(bias_acc_cov_);               // Acc bias in continuous.
  p_->setBiasOmegaCovariance(bias_omega_cov_);           // Gyro bias in continuous.
  p_->setBiasAccOmegaInt(bias_acc_omega_int_);

  imu_preintegrated_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(p_, prior_imu_bias_);

  // Look up transform from UTM to odom.
  tf::StampedTransform transform;
  static tf::TransformListener tf_listener;
  static ros::Duration wait_time = ros::Duration(100);
  tf_listener.waitForTransform("odom", "utm", ros::Time(0), wait_time);
  tf_listener.lookupTransform("odom", "utm", ros::Time(0), transform);

  oTutm_ = gtsam::Pose3(
    gtsam::Rot3::Quaternion(
      transform.getRotation().w(),
      transform.getRotation().x(),
      transform.getRotation().y(),
      transform.getRotation().z()
    ), gtsam::Point3(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z()));

  ROS_INFO_STREAM(oTutm_);

  while (ros::ok())
  {
    ros::spinOnce();  // handle subscriber callbacks
  }
}

void PoseSlamEstimator::imu_callback(const sensor_msgs::ImuConstPtr& msg)
{
  // Update initial gps and orientation readings until both are received.
  double msg_t = msg->header.stamp.sec + msg->header.stamp.nsec * 1e-9;  // Message timestamp in seconds.
  if (!fg_initialized_)
  {
    if (!initial_imu_reading_received_ || !initial_gps_coords_received_)
    {
      initial_imu_reading_received_ = true;
      last_imu_msg_t_ = msg_t;
      initial_state_(3) = msg->orientation.x;
      initial_state_(4) = msg->orientation.y;
      initial_state_(5) = msg->orientation.z;
      initial_state_(6) = msg->orientation.w;
      initial_state_(7) = 0.0;  // Assume zero initial velocity.
      initial_state_(8) = 0.0;
      initial_state_(9) = 0.0;
    }
    else
    {
      initialize_pose_graph();
    }
    return;
  }

  double dt = msg_t - last_imu_msg_t_;  // Time since last imu message.
  ROS_INFO_STREAM("dt: " << dt);
  gtsam::Vector3 measured_acc =
      (gtsam::Vector(3) << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z)
          .finished();
  gtsam::Vector3 measured_omega =
      (gtsam::Vector(3) << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z).finished();
  imu_preintegrated_->integrateMeasurement(measured_acc, measured_omega, dt);
  last_imu_msg_t_ = msg_t;

  current_orientation_ =
      gtsam::Quaternion(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
}

void PoseSlamEstimator::gps_callback(const sensor_msgs::NavSatFixConstPtr& msg)
{
  // if (!initial_gps_coords_received_)
  //   geodetic_converter_.initialiseReference(static_cast<double>(msg->latitude), static_cast<double>(msg->longitude), static_cast<double>(msg->altitude));
  if (!initial_gps_coords_received_)
    enu_.Reset(static_cast<double>(msg->latitude), static_cast<double>(msg->longitude), static_cast<double>(msg->altitude));

  // Convert gps readings to local cartesian.
  double east, north, up;
  // geodetic_converter_.geodetic2Ned(static_cast<double>(msg->latitude), static_cast<double>(msg->longitude),
  //                                  static_cast<double>(msg->altitude), &north, &east, &down);
  enu_.Forward(static_cast<double>(msg->latitude), static_cast<double>(msg->longitude),
                                   static_cast<double>(msg->altitude), east, north, up);

  ROS_INFO_STREAM("East: " << east << ", North: " << north << ", Up: " << up);


  // Update initial gps and orientation readings until both are received.
  if (!fg_initialized_)
  {
    if (!initial_imu_reading_received_ || !initial_gps_coords_received_)
    {
      initial_gps_coords_received_ = true;
      initial_state_(0) = east;
      initial_state_(1) = north;
      initial_state_(2) = up;
    }
    else
    {
      initialize_pose_graph();
    }
    return;
  }

  correction_count_++;

  // Add IMU factor to graph.
  const gtsam::PreintegratedCombinedMeasurements& preint_imu_combined =
      dynamic_cast<const gtsam::PreintegratedCombinedMeasurements&>(*imu_preintegrated_);
  gtsam::CombinedImuFactor imu_factor(X(correction_count_ - 1), V(correction_count_ - 1), X(correction_count_),
                                      V(correction_count_), B(correction_count_ - 1), B(correction_count_),
                                      preint_imu_combined);
  graph_->add(imu_factor);

  // Add GPS factor.
  gtsam::noiseModel::Diagonal::shared_ptr correction_noise =
      gtsam::noiseModel::Isotropic::Sigma(3, gps_noise_elements_);
  gtsam::GPSFactor gps_factor(X(correction_count_), gtsam::Point3(east, north, up), correction_noise);
  graph_->add(gps_factor);

  // Optimize!
  prop_state_ = imu_preintegrated_->predict(prev_state_, prev_bias_);
  initial_values_.insert(X(correction_count_), prop_state_.pose());
  initial_values_.insert(V(correction_count_), prop_state_.v());
  initial_values_.insert(B(correction_count_), prev_bias_);

  std::cout << "prop_state: [t: (" << prop_state_.pose().translation().vector().transpose()
            << "), R: (" << prop_state_.pose().rotation().rpy().transpose() << ")]" << std::endl;

  gtsam::LevenbergMarquardtOptimizer optimizer(*graph_, initial_values_);
  results_ = optimizer.optimize();

  // Overwrite the beginning of the preintegration for the next step.
  prev_state_ = gtsam::NavState(results_.at<gtsam::Pose3>(X(correction_count_)),
                                results_.at<gtsam::Vector3>(V(correction_count_)));
  prev_bias_ = results_.at<gtsam::imuBias::ConstantBias>(B(correction_count_));

  // Reset the preintegration object.
  imu_preintegrated_->resetIntegrationAndSetBias(prev_bias_);

  // Print out the position and orientation error for comparison.
  gtsam::Vector3 gtsam_position = prev_state_.pose().translation();
  gtsam::Vector3 position_error = gtsam_position - (gtsam::Vector(3) << east, north, up).finished();
  double current_position_error = position_error.norm();

  gtsam::Quaternion gtsam_quat = prev_state_.pose().rotation().toQuaternion();
  gtsam::Quaternion quat_error = gtsam_quat * current_orientation_.inverse();
  quat_error.normalize();
  gtsam::Vector3 euler_angle_error(quat_error.x() * 2, quat_error.y() * 2, quat_error.z() * 2);
  double current_orientation_error = euler_angle_error.norm();

  ROS_INFO_STREAM("Position error: " << current_position_error << "\t Angular Error: " << current_orientation_error);
  std::cout << "opt_state: [t: (" << prev_state_.pose().translation().vector().transpose()
            << "), R: (" << prev_state_.pose().rotation().rpy().transpose() << ")]" << std::endl;

  // Publish PoseSLAM trajectory.
  publish_path();
}

void PoseSlamEstimator::initialize_pose_graph()
{
  fg_initialized_ = true;
  // Assemble initial quaternion through gtsam constructor ::quaternion(w,x,y,z);
  gtsam::Rot3 prior_rotation =
      gtsam::Rot3::Quaternion(initial_state_(6), initial_state_(3), initial_state_(4), initial_state_(5));
  gtsam::Point3 prior_point(initial_state_.head<3>());
  gtsam::Pose3 prior_pose(prior_rotation, prior_point);
  gtsam::Vector3 prior_velocity(initial_state_.tail<3>());

  initial_values_.insert(X(correction_count_), prior_pose);
  initial_values_.insert(V(correction_count_), prior_velocity);
  initial_values_.insert(B(correction_count_), prior_imu_bias_);

  // Anchor the factor graph by adding all prior factors.
  graph_->add(gtsam::PriorFactor<gtsam::Pose3>(X(correction_count_), prior_pose, pose_noise_model_));
  graph_->add(gtsam::PriorFactor<gtsam::Vector3>(V(correction_count_), prior_velocity, velocity_noise_model_));
  graph_->add(
      gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(correction_count_), prior_imu_bias_, bias_noise_model_));

  prev_state_ = gtsam::NavState(prior_pose, prior_velocity);
  prop_state_ = prev_state_;
  prev_bias_ = prior_imu_bias_;
}

void PoseSlamEstimator::publish_path()
{
  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = "odom";

  for (int i = 0; i <= correction_count_; i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = path_msg.header.stamp;
    pose.header.frame_id = path_msg.header.frame_id;

    gtsam::Pose3 pose_i = results_.at<gtsam::Pose3>(X(i));
    pose.pose.position.x = pose_i.translation().vector()[0];
    pose.pose.position.y = pose_i.translation().vector()[1];
    pose.pose.position.z = pose_i.translation().vector()[2];

    pose.pose.orientation.w = pose_i.rotation().toQuaternion().w();
    pose.pose.orientation.x = pose_i.rotation().toQuaternion().x();
    pose.pose.orientation.y = pose_i.rotation().toQuaternion().y();
    pose.pose.orientation.z = pose_i.rotation().toQuaternion().z();

    path_msg.poses.push_back(pose);
  }

  trajectory_publisher_.publish(path_msg);
}
