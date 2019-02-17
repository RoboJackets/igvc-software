/**
 * This class performs an online calibration of the IMU by fitting an ellipsoid
 * to the past N samples and performing a transform.
 *
 * Responsibilities:
 * 1. Subscribe to all magnetometer data, publish calibrated version
 * 2. At the user defined rate, calibrate the magnetometer using the past N samples
 *
 * Author: Oswin So <oswinso@gatech.edu>
 * Date Created: February 16, 2019
 */

#ifndef PROJECT_IMU_CALIBRATION_H
#define PROJECT_IMU_CALIBRATION_H

#include <ros/publisher.h>
#include <sensor_msgs/MagneticField.h>
#include <boost/circular_buffer.hpp>
#include <Eigen/Dense>
#include <mutex>

class IMU_calibration {
 public:
  IMU_calibration();
 private:
  using hz = double;
  using Magnetic_Vector = Eigen::Vector3f;

  bool m_save_measurement = false;
  int m_buffer_size;
  hz m_sample_frequency;
  hz m_calibration_frequency;
  std::string m_calibrated_topic;
  std::string m_raw_topic;

  sensor_msgs::MagneticFieldConstPtr m_last_measurement;
  Eigen::Matrix3d m_correction;

  ros::Publisher m_corrected_magnetic_field_pub;
  boost::circular_buffer<Magnetic_Vector> magnetic_field_buffer;


  void callback(const sensor_msgs::MagneticFieldConstPtr &raw_msg);
  void calibrator();

};

#endif //PROJECT_IMU_CALIBRATION_H
