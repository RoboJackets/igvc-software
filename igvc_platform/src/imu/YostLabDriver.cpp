#include "YostLabDriver.h"
#include <parameter_assertions/assertions.h>

YostLabDriver::YostLabDriver(ros::NodeHandle &nh_, ros::NodeHandle &priv_nh_)
  : SerialInterface(priv_nh_), yostlab_priv_nh_(priv_nh_), yostlab_nh_(nh_)
{
  serialConnect();
  imu_pub_ = yostlab_nh_.advertise<sensor_msgs::Imu>("/imu", 10);
  updater.setHardwareIDf("IMU: %s", getSerialPort().c_str());
  updater.add("IMU Diagnostic", this, &YostLabDriver::imu_diagnostic);

  // use identity matrix as default orientation correction
  assertions::param(yostlab_priv_nh_, "imu_orientation_correction", imu_orientation_correction_,
                    std::vector<double>{ 1, 0, 0, 0, 1, 0, 0, 0, 1 });
  assertions::param(yostlab_priv_nh_, "orientation_rotation", orientation_rotation_, 0.0);
  assertions::getParam(yostlab_priv_nh_, "frame_id", frame_id_);
  yostlab_priv_nh_.param("spin_frequency", spin_frequency_, 100.0);
}

//! Destructor
YostLabDriver::~YostLabDriver()
{
  updater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "IMU Node Terminated");
}

void YostLabDriver::restoreFactorySettings()
{
  serialWriteString(RESTORE_FACTORY_SETTINGS);
}

std::string YostLabDriver::getSoftwareVersion()
{
  flush();
  serialWriteString(GET_FIRMWARE_VERSION_STRING);
  const std::string buf = serialReadLine();
  ROS_INFO_STREAM(logger << "Software version: " << buf);
  return buf;
}

void YostLabDriver::imu_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (sensor_temp_ > MAX_IMU_TEMP)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "IMU temp too high");
  }
  else if (sensor_temp_ < MIN_IMU_TEMP)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "IMU temp too low");
  }
  else if ((ros::Time::now() - lastUpdateTime_).toSec() > IMU_TIMEOUT_DELAY)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "IMU not updating");
  }
  else if (abs(quaternion_length_ - 1.0) > QUATERNION_LENGTH_TOL)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "IMU quaternion isn't normalized");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "IMU Online");
  }
  stat.add("software_version", software_version_);
  stat.add("calibration_mode", calibration_mode_);
  stat.add("mi_mode", mi_mode_);
  stat.add("axis_direction", axis_direction_);
  stat.add("imu_temp", sensor_temp_);
  double roll, pitch, yaw;
  tf::Matrix3x3(last_quat_).getRPY(roll, pitch, yaw);
  double radian_to_degrees = 180.0 / M_PI;
  stat.add("roll", roll * radian_to_degrees);
  stat.add("pitch", pitch * radian_to_degrees);
  stat.add("yaw", yaw * radian_to_degrees);
}

std::string YostLabDriver::getEulerDecomp()
{
  flush();
  serialWriteString(GET_EULER_DECOMPOSTION_ORDER);
  const std::string buf = serialReadLine();
  const std::string ret_buf = [&]() {
    if (buf == "0\r\n")
      return "XYZ";
    else if (buf == "1\r\n")
      return "YZX";
    else if (buf == "2\r\n")
      return "ZXY";
    else if (buf == "3\r\n")
      return "ZYX";
    else if (buf == "4\r\n")
      return "XZY";
    else if (buf == "5\r\n")
      return "YXZ";
    else
      return "Unknown";
  }();
  ROS_INFO_STREAM(logger << "Euler Decomposition: " << ret_buf << ", buf is: " << buf);
  return ret_buf;
}

std::string YostLabDriver::getAxisDirection()
{
  flush();
  serialWriteString(GET_AXIS_DIRECTION);
  const std::string buf = serialReadLine();
  const std::string ret_buf = [&]() {
    if (buf == "0\r\n")
      return "X: Right, Y: Up, Z: Forward";
    else if (buf == "1\r\n")
      return "X: Right, Y: Forward, Z: Up";
    else if (buf == "2\r\n")
      return "X: Up, Y: Right, Z: Forward";
    else if (buf == "3\r\n")
      return "X: Forward, Y: Right, Z: Up";
    else if (buf == "4\r\n")
      return "X: Up, Y: Forward, Z: Right";
    else if (buf == "5\r\n")
      return "X: Forward, Y: Up, Z: Right";
    else
      return "Unknown";
  }();
  ROS_INFO_STREAM(logger << "Axis Direction: " << ret_buf << ", buf is: " << buf);
  return ret_buf;
}

void YostLabDriver::startGyroCalibration()
{
  flush();
  ROS_INFO_STREAM(logger << "Starting Auto Gyro Calibration");
  serialWriteString(BEGIN_GYRO_AUTO_CALIB);
  ros::Duration(5.0).sleep();
}

void YostLabDriver::setMIMode(bool on)
{
  if (on)
    serialWriteString(SET_MI_MODE_ENABLED);
  else
    serialWriteString(SET_MI_MODE_DISABLED);
}

std::string YostLabDriver::getCalibMode()
{
  flush();
  serialWriteString(GET_CALIB_MODE);
  const std::string buf = serialReadLine();
  const std::string ret_buf = [&]() {
    if (buf == "0\r\n")
      return "Bias";
    else if (buf == "1\r\n")
      return "Scale and Bias";
    else
      return "Unknown";
  }();
  ROS_INFO_STREAM(logger << "Calibration Mode: " << ret_buf << ", buf is: " << buf);
  return ret_buf;
}

std::string YostLabDriver::getMIMode()
{
  flush();
  serialWriteString(GET_MI_MODE_ENABLED);
  const std::string buf = serialReadLine();
  const std::string ret_buf = [&]() {
    if (buf == "0\r\n")
      return "Disabled";
    else if (buf == "1\r\n")
      return "Enabled";
    else
      return "Unknown";
  }();
  ROS_INFO_STREAM(logger << "MI Mode: " << ret_buf << ", buf is: " << buf);
  return ret_buf;
}

void YostLabDriver::run()
{
  setAndCheckIMUSettings();

  // Performs auto-gyroscope calibration. Sensor should remain still while samples are taken.
  startGyroCalibration();

  // commit settings and start streaming
  flush();
  serialWriteString(SET_STREAMING_TIMING_5_MS);
  serialWriteString(START_STREAMING);

  ros::Rate loop_rate(spin_frequency_);  // Hz

  int line_num_ = 0;
  std::vector<double> parsed_val;

  while (ros::ok())
  {
    while (available() > 0)
    {
      line_num_ += 1;
      // Parses the line and
      auto terms_count = addToParsedVals(serialReadLine(), parsed_val);
      // Checks for a single a line containing a single number (IMU temp) which signifies the end of the imu message
      if (terms_count == 1)
      {
        // Verify that it is a complete message of 4 lines
        if (line_num_ == 4)
        {
          createAndPublishIMUMessage(parsed_val);
        }
        else
        {
          ROS_WARN_STREAM("Incomplete message from IMU. Throwing it away.");
        }
        parsed_val.clear();
        line_num_ = 0;
      }
    }
    updater.update();
    loop_rate.sleep();
    ros::spinOnce();
  }
}

void YostLabDriver::setAndCheckIMUSettings()
{
  flush();
  setTimeout(500);
  serialWriteString(SET_AXIS_DIRECTIONS);
  serialWriteString(SET_STREAMING_SLOTS);
  serialWriteString(COMMIT_SETTINGS);

  // small delay to allow imu to commit its settings before we read them back
  ros::Duration(0.02).sleep();

  // print/debug statements
  software_version_ = getSoftwareVersion();
  axis_direction_ = getAxisDirection();
  std::string euler = getEulerDecomp();
  calibration_mode_ = getCalibMode();
  mi_mode_ = getMIMode();
}

void YostLabDriver::createAndPublishIMUMessage(std::vector<double> &parsed_val)
{
  // orientation correction matrices in 3x3 row-major format and quaternion
  static const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> correction_mat(imu_orientation_correction_.data());
  static const Eigen::Quaternion<double> correction_mat_quat(correction_mat);
  static const tf::Quaternion rot = tf::createQuaternionFromYaw(orientation_rotation_);

  sensor_msgs::Imu imu_msg;
  imu_msg.header.seq = 0;
  imu_msg.header.stamp = ros::Time::now();
  imu_msg.header.seq++;
  imu_msg.header.frame_id = frame_id_;

  // construct quaternion with (x,y,z,w)
  tf::Quaternion quat{ parsed_val[0], parsed_val[1], parsed_val[2], parsed_val[3] };
  quaternion_length_ = tf::length(quat);
  quat = rot * quat;

  // Filtered orientation estimate
  tf::quaternionTFToMsg(quat, imu_msg.orientation);
  imu_msg.orientation_covariance = { .1, 0, 0, 0, .1, 0, 0, 0, .1 };

  // Corrected angular velocity.
  Eigen::Vector3d angular_vel_raw(parsed_val[4], parsed_val[5], parsed_val[6]);
  angular_vel_raw = correction_mat * angular_vel_raw;

  // Corrected linear acceleration.
  Eigen::Vector3d linear_accel_raw(parsed_val[7], parsed_val[8], parsed_val[9]);
  linear_accel_raw = correction_mat * linear_accel_raw * GRAVITY;

  imu_msg.angular_velocity.x = angular_vel_raw[0];
  imu_msg.angular_velocity.y = angular_vel_raw[1];
  imu_msg.angular_velocity.z = angular_vel_raw[2];
  imu_msg.angular_velocity_covariance = { .1, 0, 0, 0, .1, 0, 0, 0, .07 };

  imu_msg.linear_acceleration.x = linear_accel_raw[0];
  imu_msg.linear_acceleration.y = linear_accel_raw[1];
  imu_msg.linear_acceleration.z = linear_accel_raw[2];
  imu_msg.linear_acceleration_covariance = { .1, 0, 0, 0, .1, 0, 0, 0, .1 };

  sensor_temp_ = parsed_val[10];

  imu_pub_.publish(imu_msg);
  lastUpdateTime_ = ros::Time::now();
  last_quat_ = quat;
}

int YostLabDriver::addToParsedVals(const std::string &buf, std::vector<double> &parsed_vals)
{
  std::stringstream ss(buf);
  int terms_count = 0;
  double i;
  while (ss >> i)
  {
    parsed_vals.push_back(i);
    terms_count++;
    if (ss.peek() == ',')
      ss.ignore();
  }
  return terms_count;
}