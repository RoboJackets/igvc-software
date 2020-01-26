#include "YostLabDriver.h"
#include <parameter_assertions/assertions.h>

YostLabDriver::YostLabDriver(ros::NodeHandle& nh_, ros::NodeHandle& priv_nh_)
  : SerialInterface(priv_nh_), yostlab_priv_nh_(priv_nh_), yostlab_nh_(nh_)
{
  this->SerialConnect();
  this->imu_pub_ = this->yostlab_nh_.advertise<sensor_msgs::Imu>("/imu", 10);
  this->updater.setHardwareIDf("IMU: %s", this->getSerialPort().c_str());
  this->updater.add("IMU Diagnostic", this, &YostLabDriver::imu_diagnostic);

  // use identity matrix as default orientation correction
  assertions::param(this->yostlab_priv_nh_, "imu_orientation_correction", this->imu_orientation_correction_,
                    std::vector<double>{ 1, 0, 0, 0, 1, 0, 0, 0, 1 });
  assertions::param(yostlab_priv_nh_, "orientation_rotation", orientation_rotation_, 0.0);
  assertions::getParam(this->yostlab_priv_nh_, "frame_id", this->frame_id_);
}

//! Destructor
YostLabDriver::~YostLabDriver(){
    this->running = false;
    this->updater.force_update();
}

void YostLabDriver::restoreFactorySettings()
{
  this->SerialWriteString(RESTORE_FACTORY_SETTINGS);
}
const std::string YostLabDriver::getSoftwareVersion()
{
  this->SerialWriteString(GET_FIRMWARE_VERSION_STRING);
  const std::string buf = this->SerialReadLine();
  ROS_INFO_STREAM(this->logger << "Software version: " << buf);
  return buf;
}

void YostLabDriver::imu_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    if (running){
        if(sensor_temp > 185){
            stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "IMU temp too high");
        } else if (sensor_temp < -40){
            stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "IMU temp too low");
        } else {
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "IMU Online");
        }
    } else {
        stat.summary(diagnostic_msgs::DiagnosticStatus::STALE, "IMU Stopped Running");
    }
    stat.add("software_version", software_version);
    stat.add("calibration_mode", calibration_mode);
    stat.add("mi_mode", mi_mode);
    stat.add("axis_direction", axis_direction);
}

const std::string YostLabDriver::getEulerDecomp()
{
  this->SerialWriteString(GET_EULER_DECOMPOSTION_ORDER);
  const std::string buf = this->SerialReadLine();
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
  ROS_INFO_STREAM(this->logger << "Euler Decomposition: " << ret_buf << ", buf is: " << buf);
  return ret_buf;
}

const std::string YostLabDriver::getAxisDirection()
{
  this->SerialWriteString(GET_AXIS_DIRECTION);
  const std::string buf = this->SerialReadLine();
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
  ROS_INFO_STREAM(this->logger << "Axis Direction: " << ret_buf << ", buf is: " << buf);
  return ret_buf;
}

void YostLabDriver::startGyroCalibration(void)
{
  ROS_INFO_STREAM(this->logger << "Starting Auto Gyro Calibration");
  this->SerialWriteString(BEGIN_GYRO_AUTO_CALIB);
  ros::Duration(5.0).sleep();
}

void YostLabDriver::setMIMode(bool on)
{
  if (on)
    this->SerialWriteString(SET_MI_MODE_ENABLED);
  else
    this->SerialWriteString(SET_MI_MODE_DISABLED);
}

const std::string YostLabDriver::getCalibMode()
{
  this->SerialWriteString(GET_CALIB_MODE);
  const std::string buf = this->SerialReadLine();
  const std::string ret_buf = [&]() {
    if (buf == "0\r\n")
      return "Bias";
    else if (buf == "1\r\n")
      return "Scale and Bias";
    else
      return "Unknown";
  }();
  ROS_INFO_STREAM(this->logger << "Calibration Mode: " << ret_buf << ", buf is: " << buf);
  return ret_buf;
}

const std::string YostLabDriver::getMIMode()
{
  this->SerialWriteString(GET_MI_MODE_ENABLED);
  const std::string buf = this->SerialReadLine();
  const std::string ret_buf = [&]() {
    if (buf == "0\r\n")
      return "Disabled";
    else if (buf == "1\r\n")
      return "Enabled";
    else
      return "Unknown";
  }();
  ROS_INFO_STREAM(this->logger << "MI Mode: " << ret_buf << ", buf is: " << buf);
  return ret_buf;
}

//! Run the serial sync
void YostLabDriver::run()
{
  // Performs auto-gyroscope calibration. Sensor should remain still while samples are taken.
  this->startGyroCalibration();

  /*
  Slot #1: untared orientation as quaternion [4x float]
  Slot #2: corrected gyroscope vector [3x float]
  Slot #3: corrected acceleration vector [3x float]
  Slot #4: sensor temp in ºF
  Slot #[4-8]: No Command
  */
  this->SerialWriteString(SET_AXIS_DIRECTIONS);
  this->SerialWriteString(SET_STREAMING_SLOTS);

  // commit settings and start streaming!
  this->SerialWriteString(COMMIT_SETTINGS);
  this->SerialWriteString(SET_STREAMING_TIMING_5_MS);
  this->SerialWriteString(START_STREAMING);

  // print/debug statements
  software_version = this->getSoftwareVersion();
  axis_direction = this->getAxisDirection();
  this->getEulerDecomp();
  calibration_mode = this->getCalibMode();
  mi_mode = this->getMIMode();

  ros::Rate loop_rate(20);  // 20Hz

  int line_num_ = 0;
  sensor_msgs::Imu imu_msg_;
  imu_msg_.header.seq = 0;
  std::vector<double> parsed_val_;

  // orientation correction matrices in 3x3 row-major format and quaternion
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> correction_mat(imu_orientation_correction_.data());
  Eigen::Quaternion<double> correction_mat_quat(correction_mat);

  while (ros::ok())
  {
      this->running = true;
    while (this->Available() > 0)
    {
      line_num_ += 1;
      std::string buf = this->SerialReadLine();
      std::string parse_buf_;
      std::stringstream ss(buf);
      double i;
      while (ss >> i)
      {
        parsed_val_.push_back(i);
        if (ss.peek() == ',')
          ss.ignore();
      }

      if (line_num_ == 4)
      {
        line_num_ = 0;
        imu_msg_.header.stamp = ros::Time::now();
        imu_msg_.header.seq++;
        imu_msg_.header.frame_id = frame_id_;

        // construct quaternion with (x,y,z,w)
        tf::Quaternion quat{ parsed_val_[0], parsed_val_[1], parsed_val_[2], parsed_val_[3] };

        tf::Quaternion rot = tf::createQuaternionFromYaw(orientation_rotation_);
        quat = rot * quat;

        // Filtered orientation estimate
        tf::quaternionTFToMsg(quat, imu_msg_.orientation);
        imu_msg_.orientation_covariance = { .1, 0, 0, 0, .1, 0, 0, 0, .1 };

        // Corrected angular velocity.
        Eigen::Vector3d angular_vel_raw(parsed_val_[4], parsed_val_[5], parsed_val_[6]);
        angular_vel_raw = correction_mat * angular_vel_raw;

        // Corrected linear acceleration.
        Eigen::Vector3d linear_accel_raw(parsed_val_[7], parsed_val_[8], parsed_val_[9]);
        linear_accel_raw = correction_mat * linear_accel_raw * GRAVITY;

        imu_msg_.angular_velocity.x = angular_vel_raw[0];
        imu_msg_.angular_velocity.y = angular_vel_raw[1];
        imu_msg_.angular_velocity.z = angular_vel_raw[2];
        imu_msg_.angular_velocity_covariance = { .1, 0, 0, 0, .1, 0, 0, 0, .07 };

        imu_msg_.linear_acceleration.x = linear_accel_raw[0];
        imu_msg_.linear_acceleration.y = linear_accel_raw[1];
        imu_msg_.linear_acceleration.z = linear_accel_raw[2];
        imu_msg_.linear_acceleration_covariance = { .1, 0, 0, 0, .1, 0, 0, 0, .1 };

        sensor_temp = parsed_val_[10];

        parsed_val_.clear();
        this->imu_pub_.publish(imu_msg_);

        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
      }
    }
    loop_rate.sleep();
    ros::spinOnce();
    this->updater.update();
  }
  this->running = false;
  this->updater.force_update();
}
