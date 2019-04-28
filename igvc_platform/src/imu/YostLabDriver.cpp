#include "YostLabDriver.h"

YostLabDriver::YostLabDriver(ros::NodeHandle& nh_, ros::NodeHandle& priv_nh_)
  : SerialInterface(priv_nh_), yostlab_priv_nh_(priv_nh_), yostlab_nh_(nh_)
{
  this->SerialConnect();
  this->imu_pub_ = this->yostlab_nh_.advertise<sensor_msgs::Imu>("/imu", 10);
  // use identity matrix as default orientation correction
  igvc::param(this->yostlab_priv_nh_, "imu_orientation_correction", this->imu_orientation_correction_,
              std::vector<double>{ 1, 0, 0, 0, 1, 0, 0, 0, 1 });
  igvc::getParam(this->yostlab_priv_nh_, "frame_id", this->frame_id_);
}

//! Destructor
YostLabDriver::~YostLabDriver()
{
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

  // print/debug statements
  this->getSoftwareVersion();
  this->getAxisDirection();
  this->getEulerDecomp();
  this->getCalibMode();
  this->getMIMode();

  /*
  Slot #1: untared orientation as quaternion [4x float]
  Slot #2: corrected gyroscope vector [3x float]
  Slot #3: corrected acceleration vector [3x float]
  Slot #4: sensor temp in ºF
  Slot #[4-8]: No Command
  */
  this->SerialWriteString(SET_STREAMING_SLOTS);

  // commit settinga and start streaming!
  this->SerialWriteString(SET_STREAMING_TIMING_5_MS);
  this->SerialWriteString(START_STREAMING);

  ros::Rate loop_rate(20);  // 20Hz

  int line_num_ = 0;
  sensor_msgs::Imu imu_msg_;
  imu_msg_.header.seq=0;
  std::vector<double> parsed_val_;

  // orientation correction matrices in 3x3 row-major format and quaternion
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> correction_mat(imu_orientation_correction_.data());
  Eigen::Quaternion<double> correction_mat_quat(correction_mat);

  while (ros::ok())
  {
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
        tf::Quaternion q(parsed_val_[0], parsed_val_[1], parsed_val_[2], parsed_val_[3]);

        // Filtered orientation estimate
        imu_msg_.orientation.x = q[0];
        imu_msg_.orientation.y = q[1];
        imu_msg_.orientation.z = q[2];
        imu_msg_.orientation.w = q[3];
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

        double sensor_temp = parsed_val_[10];

        parsed_val_.clear();
        this->imu_pub_.publish(imu_msg_);

        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        ROS_INFO_THROTTLE(1.0, "[YostLabImuDriver] R: %f, P: %f, Y: %f -- IMU Temp: %f F ", roll, pitch, yaw,
                          sensor_temp);
      }
    }
    loop_rate.sleep();
    ros::spinOnce();
  }
}
