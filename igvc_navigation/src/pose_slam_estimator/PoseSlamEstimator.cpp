#include "PoseSlamEstimator.h"
#include <parameter_assertions/assertions.h>
#include <igvc_utils/NodeUtils.hpp>

PoseSlamEstimator::PoseSlamEstimator(ros::NodeHandle* nodehandle) : nh_(*nodehandle)
{
  ros::NodeHandle pNh("~");

  // Subscribe to sensor readings.
  imu_sub_ = nh_.subscribe("/imu", 1, &PoseSlamEstimator::imu_callback, this);
  gps_sub_ = nh_.subscribe("/fix", 1, &PoseSlamEstimator::gps_callback, this);

  // Publish estimated trajectory.
  trajectory_publisher_ = nh_.advertise<nav_msgs::Path>("/pose_slam_trajectory", 1);

  p_->setAccelerometerCovariance(measured_acc_cov_);
  p_->setIntegrationCovariance(integration_error_cov_);
  p_->setGyroscopeCovariance(measured_omega_cov_);
  p_->setBiasAccCovariance(bias_acc_cov_);
  p_->setBiasOmegaCovariance(bias_omega_cov_);
  p_->setBiasAccOmegaInt(bias_acc_omega_int_);

  imu_preintegrated_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(p_, prior_imu_bias_);

  // assertions::getParam(pNh, "c_space", configuration_space_);

  // ros::Rate rate(rate_);  // path update rate

  while (ros::ok())
  {
    ros::spinOnce();  // handle subscriber callbacks

    // ROS_INFO_STREAM_COND(num_nodes_updated > 0, num_nodes_updated << " nodes updated");
    // rate.sleep();
  }
}

void PoseSlamEstimator::imu_callback(const sensor_msgs::ImuConstPtr& msg)
{
  // Update initial gps and orientation readings until both are received.
  if (!fg_initialized_)
  {
    if (!initial_imu_reading_received_ || !initial_gps_coords_received)
    {
      initial_imu_reading_received_ = true;
      last_imu_msg_t_ = msg->header.stamp.sec;
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

  double dt = msg->header.stamp.sec - last_imu_msg_t_;

  gtsam::Vector3 measured_acc =
      (gtsam::Vector(3) << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z)
          .finished();
  gtsam::Vector3 measured_omega =
      (gtsam::Vector(3) << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z).finished();
  imu_preintegrated_->integrateMeasurement(measured_acc, measured_omega, dt);

  last_imu_msg_t_ = msg->header.stamp.sec;
}

void PoseSlamEstimator::gps_callback(const sensor_msgs::NavSatFixConstPtr& msg)
{
  // Convert gps readings to ned.
  double north, east, down;
  geodetic_converter_.geodetic2Ned(static_cast<double>(msg->latitude), static_cast<double>(msg->longitude),
                                   static_cast<double>(msg->altitude), &north, &east, &down);

  // Update initial gps and orientation readings until both are received.
  if (!fg_initialized_)
  {
    if (!initial_imu_reading_received_ || !initial_gps_coords_received)
    {
      initial_gps_coords_received = true;
      initial_state_(0) = north;
      initial_state_(1) = east;
      initial_state_(2) = down;
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
  gtsam::noiseModel::Diagonal::shared_ptr correction_noise = gtsam::noiseModel::Isotropic::Sigma(3, 1.0);
  gtsam::GPSFactor gps_factor(X(correction_count_), gtsam::Point3(north, east, down), correction_noise);
  graph_->add(gps_factor);

  // Optimize!
  prop_state_ = imu_preintegrated_->predict(prev_state_, prev_bias_);
  initial_values_.insert(X(correction_count_), prop_state_.pose());
  initial_values_.insert(V(correction_count_), prop_state_.v());
  initial_values_.insert(B(correction_count_), prev_bias_);

  gtsam::LevenbergMarquardtOptimizer optimizer(*graph_, initial_values_);
  results_ = optimizer.optimize();

  // Overwrite the beginning of the preintegration for the next step.
  prev_state_ = gtsam::NavState(results_.at<gtsam::Pose3>(X(correction_count_)),
                                results_.at<gtsam::Vector3>(V(correction_count_)));
  prev_bias_ = results_.at<gtsam::imuBias::ConstantBias>(B(correction_count_));

  // Reset the preintegration object.
  imu_preintegrated_->resetIntegrationAndSetBias(prev_bias_);

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
