// This node reads and publishes IMU data from the FSM-9 IMU.

#include <Eigen/Dense>
#include <freespace/freespace.h>
#include <freespace/freespace_util.h>
#include <geometry_msgs/Vector3.h>
#include <igvc_utils/NodeUtils.hpp>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <vector>

/** A function for initializing a single FSM-9 IMU. This function sets up the IMU
  * such that it returns expected measurements (ex. linear acceleration, angular velocity, etc.).
  * \param ids an array for storing the identified IMU device ID.
  */ 
int initializeIMU(FreespaceDeviceId* ids)
{
  int ret = freespace_init();
  if (ret != FREESPACE_SUCCESS)
  {
    ROS_ERROR_STREAM("Failed to init IMU");
    return 0;
  }

  // Initialize freespace device (we expect only 1).
  int num_found = 0;
  ret = freespace_getDeviceList(ids, 1, &num_found);
  if (num_found <= 0)
  {
    ROS_ERROR_STREAM("failed to connect to IMU found no devices");
    return 0;
  }
  ROS_INFO_STREAM("number of devices = " << num_found);

  ret = freespace_openDevice(ids[0]);
  if (ret != FREESPACE_SUCCESS)
  {
    ROS_ERROR_STREAM("failure to connect to device " << ret);
    return 0;
  }

  freespace_message request;
  request.messageType = FREESPACE_MESSAGE_DATAMODECONTROLV2REQUEST;
  request.dataModeControlV2Request.operatingStatus = 0;  // 0 to change 1 to read
  request.dataModeControlV2Request.outputStatus = 0;     // 0 to change 1 to read
  request.dataModeControlV2Request.mode = 4;             // Full Motion On
  request.dataModeControlV2Request.packetSelect = 8;     // MotionEngine Output
  request.dataModeControlV2Request.formatSelect = 0;     // Format 0
  request.dataModeControlV2Request.ff0 = 0;              // Enable Pointer
  request.dataModeControlV2Request.ff1 = 1;              // Enable Linear Acceleration
  request.dataModeControlV2Request.ff2 = 1;              // Enable Linear Acceleration, No Gravity
  request.dataModeControlV2Request.ff3 = 1;              // Enable Angular Velocity
  request.dataModeControlV2Request.ff4 = 1;              // Enable Magnetometer
  request.dataModeControlV2Request.ff5 = 0;              // Enable Temperature
  request.dataModeControlV2Request.ff6 = 1;              // Enable Angular Position (WXYZ)
  request.dataModeControlV2Request.ff7 = 0;              // Nothing
  ret = freespace_sendMessage(ids[0], &request);
  if (ret != FREESPACE_SUCCESS)
  {
    ROS_ERROR_STREAM("failed to send message");
  }

  return ret;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu");

  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  // Get matrix to correct IMU orientation.
  std::vector<double> imu_orientation_correction;
  igvc::getParam(nh, "imu_orientation_correction", imu_orientation_correction);

  // Get threshold value for triggering a warning when the deviation between the filtered
  // heading and magnetometer heading becomes too large.
  double heading_dev_thresh_deg;
  igvc::getParam(nh, "heading_dev_thresh_deg", heading_dev_thresh_deg);

  // Get yaw offset for making yaw relative to East.
  double yaw_offset;
  igvc::getParam(nh, "yaw_offset", yaw_offset);

  // There should be 9 elements (3x3 matrix).
  if (imu_orientation_correction.size() != 9) {
    ROS_ERROR_STREAM("IMU orientation correction matrix does not have 9 elements (3x3).");
    return 0;
  }

  // Populate an Eigen matrix.
  Eigen::Matrix3d correction_mat(imu_orientation_correction.data());

  // Publish imu data without gravity for EKF localization.
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu", 1000);

  // Publish imu data with gravity for factor graph implementation.
  ros::Publisher imu_raw_pub = nh.advertise<sensor_msgs::Imu>("/imu_raw", 1000);

  // sequence of published messages - should be monotonically increasing.
  int seq = 0;

  // Initialize IMU.
  FreespaceDeviceId ids[1];
  int ret = initializeIMU(ids);
  if (ret != FREESPACE_SUCCESS) {
    return 0;
  }

  // Variable to store data that is read from the IMU.
  freespace_message response;

  while (ros::ok())
  {
    ros::spinOnce();

    // Read the IMU message every loop. Timeout after 1000 milliseconds.
    ret = freespace_readMessage(ids[0], &response, 1000);
    if (ret != FREESPACE_SUCCESS)
    {
      if (ret == FREESPACE_ERROR_TIMEOUT)
      {
        ROS_ERROR_STREAM("failed to read IMU TIMEOUT");
      }
      else
      {
        ROS_ERROR_STREAM("failed to read IMU" << ret);
      }
    }

    if (response.messageType != FREESPACE_MESSAGE_MOTIONENGINEOUTPUT)
    {
      ROS_ERROR_STREAM("IMU error: got unknown message type " << response.messageType);
    }

    // Get accelerometer measurements.
    MultiAxisSensor accel_msg;
    MultiAxisSensor accel_raw_msg;

    ret = freespace_util_getAccNoGravity(&response.motionEngineOutput, &accel_msg);
    if (ret != FREESPACE_SUCCESS)
    {
      ROS_ERROR_STREAM("failed to read acceleration no gravity." << ret);
    }

    ret = freespace_util_getAcceleration(&response.motionEngineOutput, &accel_raw_msg);
    if (ret != FREESPACE_SUCCESS)
    {
      ROS_ERROR_STREAM("failed to read acceleration." << ret);
    }

    Eigen::Vector3d accel_raw_vec(accel_raw_msg.x, accel_raw_msg.y, accel_raw_msg.z);
    accel_raw_vec = correction_mat * accel_raw_vec;

    Eigen::Vector3d accel_vec(accel_msg.x, accel_msg.y, accel_msg.z);
    accel_vec = correction_mat * accel_vec;

    MultiAxisSensor orientation_msg;
    ret = freespace_util_getAngPos(&response.motionEngineOutput, &orientation_msg);
    if (ret != FREESPACE_SUCCESS)
    {
      ROS_ERROR_STREAM("failed to read angular position " << ret);
    }
    tf::Quaternion quaternion_raw(orientation_msg.x, orientation_msg.y, orientation_msg.z, orientation_msg.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(quaternion_raw).getRPY(roll, pitch, yaw);
    yaw += yaw_offset;

    // Get gyroscope measurements.
    MultiAxisSensor angular_vel_msg;
    ret = freespace_util_getAngularVelocity(&response.motionEngineOutput, &angular_vel_msg);
    if (ret != FREESPACE_SUCCESS)
    {
      ROS_ERROR_STREAM("failed to read angular velocity " << ret);
    }

    Eigen::Vector3d angular_vel_vec(angular_vel_msg.x, angular_vel_msg.y, angular_vel_msg.z);
    angular_vel_vec = correction_mat * angular_vel_vec;

    // Get magnetometer measurement and convert to heading.
    MultiAxisSensor magnetometer_msg;
    ret = freespace_util_getMagnetometer(&response.motionEngineOutput, &magnetometer_msg);
    if (ret != FREESPACE_SUCCESS)
    {
      ROS_ERROR_STREAM("failed to read raw magnetometer values" << ret);
    }

    // Calculate heading with magnetometer reading.
    double yaw_mag = atan2(magnetometer_msg.y, magnetometer_msg.x) + yaw_offset;

    if (fabs(yaw - yaw_mag) > heading_dev_thresh_deg * M_PI / 180.0)
    {
      ROS_WARN("Magnetometer heading and filtered yaw measurement disagree by > %f degrees.", heading_dev_thresh_deg);
    }

    // Publish sensor messages with corrected transformation.
    sensor_msgs::Imu msg;
    msg.header.frame_id = "imu";
    msg.header.stamp = ros::Time::now();
    msg.header.seq = seq++;

    // TODO: Tune values, parameterize.
    msg.linear_acceleration.x = accel_vec[0];
    msg.linear_acceleration.y = accel_vec[1];
    msg.linear_acceleration.z = accel_vec[2];
    msg.linear_acceleration_covariance = { 0.01, 1e-6, 1e-6,
                                           1e-6, 0.01, 1e-6,
                                           1e-6, 1e-6, 0.01 };

    msg.angular_velocity.x = angular_vel_vec[0];
    msg.angular_velocity.y = angular_vel_vec[1];
    msg.angular_velocity.z = angular_vel_vec[2];
    msg.angular_velocity_covariance = { 0.02, 1e-6, 1e-6,
                                        1e-6, 0.02, 1e-6,
                                        1e-6, 1e-6, 0.02 };


    // This assumes flat ground and uses magnetometer for absolute heading.
    // TODO: This assumption may not hold when we switch to 3D lidar, and stop using flat ground assumption
    tf::Quaternion quaternion_mag;
    quaternion_mag.setRPY(0,0,yaw_mag);

    geometry_msgs::Quaternion orientation;
    orientation.x = quaternion_mag.x();
    orientation.y = quaternion_mag.y();
    orientation.z = quaternion_mag.z();
    orientation.w = quaternion_mag.w();

    // TODO: Tune and parameterize.
    msg.orientation = orientation;
    msg.orientation_covariance = { 0.0025, 1e-6, 1e-6,
                                   1e-6, 0.0025, 1e-6,
                                   1e-6, 1e-6, 0.0025 };

    imu_pub.publish(msg);

    // Publish raw sensor messages.
    sensor_msgs::Imu msg_raw;
    msg_raw.header.frame_id = "imu";
    msg_raw.header.stamp = msg.header.stamp;
    msg_raw.header.seq = seq;

    msg_raw.linear_acceleration.x = accel_raw_vec[0];
    msg_raw.linear_acceleration.y = accel_raw_vec[1];
    msg_raw.linear_acceleration.z = accel_raw_vec[2];
    msg_raw.linear_acceleration_covariance = { 0.005, 1e-6, 1e-6,
                                               1e-6, 0.005, 1e-6,
                                               1e-6, 1e-6, 0.005 };

    msg_raw.angular_velocity.x = angular_vel_vec[0];
    msg_raw.angular_velocity.y = angular_vel_vec[1];
    msg_raw.angular_velocity.z = angular_vel_vec[2];
    msg_raw.angular_velocity_covariance = { 0.02, 1e-6, 1e-6,
                                            1e-6, 0.02, 1e-6,
                                            1e-6, 1e-6, 0.02 };

    msg_raw.orientation = orientation;
    msg_raw.orientation_covariance = { 0.0025, 1e-6, 1e-6,
                                       1e-6, 0.0025, 1e-6,
                                       1e-6, 1e-6, 0.0025 };

    imu_raw_pub.publish(msg_raw);
  }
  return 0;
}
