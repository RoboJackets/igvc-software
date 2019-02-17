#include "imu_calibration.h"

#include <ros/ros.h>
#include <igvc_utils/NodeUtils.hpp>
#include <sensor_msgs/MagneticField.h>
#include <thread>

IMU_calibration::IMU_calibration() {
  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  // How many seconds in the past to calibrate with
//  igvc::getParam(nh, "calibration_window_size", m_orientation_transform);
  igvc::getParam(nh, "buffer_size", m_buffer_size);
  igvc::getParam(nh, "sample_frequency", m_sample_frequency);
  igvc::getParam(nh, "calibration_frequency", m_calibration_frequency);
  igvc::getParam(nh, "topics/raw_magnetic_field", m_raw_topic);
  igvc::getParam(nh, "topics/calibrated_magnetic_field", m_calibrated_topic);

  m_corrected_magnetic_field_pub = nh.advertise<sensor_msgs::MagneticField>(m_calibrated_topic, 1000);
  nh.subscribe<sensor_msgs::MagneticField>(m_raw_topic, 1, &IMU_calibration::callback, this);

  m_correction = Eigen::Matrix3d::Identity();

  std::thread thread(&IMU_calibration::calibrator, this);

  // Initialize IMU.
  ros::spin();
}

void IMU_calibration::calibrator() {
  ros::Rate r(m_calibration_frequency);
}

void IMU_calibration::callback(const sensor_msgs::MagneticFieldConstPtr &raw_msg) {
//  magnetic_field_buffer.push_back(raw_magnetic_field);
  if (m_save_measurement)
  {
    m_save_measurement = false;
    m_last_measurement = raw_msg;
  }
  Eigen::Matrix3d vec;
  vec << raw_msg->magnetic_field.x, raw_msg->magnetic_field.y, raw_msg->magnetic_field.z;
  vec = vec * m_correction;
  sensor_msgs::MagneticField corrected_msg{};
  corrected_msg.magnetic_field.x = vec(0);
  corrected_msg.magnetic_field.y = vec(1);
  corrected_msg.magnetic_field.z = vec(2);

  corrected_msg.header = raw_msg->header;
  corrected_msg.magnetic_field_covariance = raw_msg->magnetic_field_covariance;

  m_corrected_magnetic_field_pub.publish(corrected_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu");
  IMU_calibration calibration;
}

