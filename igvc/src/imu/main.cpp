#include <ros/publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <freespace/freespace.h>
#include <freespace/freespace_util.h>
#include <geometry_msgs/Vector3.h>
#include <Eigen/Dense>
#include <std_msgs/Float64.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu");

  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  double yaw_offset;
  pNh.param("yaw_offset" , yaw_offset, 0.0);

  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu", 1000);

  ros::Publisher raw_mag_pub = nh.advertise<geometry_msgs::Vector3>("/mag_raw", 1000);
  ros::Publisher raw_yaw_pub = nh.advertise<std_msgs::Float64>("/raw_yaw", 1);

  int ret = freespace_init();
  if(ret != FREESPACE_SUCCESS) {
    ROS_ERROR_STREAM("Failed to init IMU");
    return 0;
  }

  FreespaceDeviceId ids[5];
  int num_found = 0;
  ret = freespace_getDeviceList(ids, 5, &num_found);
  if(num_found <= 0) {
    ROS_ERROR_STREAM("failed to connect to IMU found no devices");
    return 0;
  }
  ROS_INFO_STREAM("number of devices = " << num_found);

  ret = freespace_openDevice(ids[0]);
  if(ret != FREESPACE_SUCCESS) {
    ROS_ERROR_STREAM("failure to connect to device " << ret);
  }

  int seq = 0;  // sequence of published messgaes - should be monatomicly increasing

  freespace_message request;
  request.messageType = FREESPACE_MESSAGE_DATAMODECONTROLV2REQUEST;
  request.dataModeControlV2Request.operatingStatus = 0; // 0 to change 1 to read
  request.dataModeControlV2Request.outputStatus = 0; // 0 to change 1 to read
  request.dataModeControlV2Request.mode = 4; // Full Motion On
  request.dataModeControlV2Request.packetSelect = 8; // MotionEngine Output
  request.dataModeControlV2Request.formatSelect = 0; // Format 0
  request.dataModeControlV2Request.ff0 = 0; // Enable Pointer
  request.dataModeControlV2Request.ff1 = 1; // Enable Linear Acceleration
  request.dataModeControlV2Request.ff2 = 1; // Enable Linear Acceleration, No Gravity
  request.dataModeControlV2Request.ff3 = 1; // Enable Angular Velocity
  request.dataModeControlV2Request.ff4 = 1; // Enable Magnetometer
  request.dataModeControlV2Request.ff5 = 0; // Enable Temperature
  request.dataModeControlV2Request.ff6 = 1; // Enable Angular Position (WXYZ)
  request.dataModeControlV2Request.ff7 = 0; // Nothing
  ret = freespace_sendMessage(ids[0], &request);
  if(ret != FREESPACE_SUCCESS) {
    ROS_ERROR_STREAM("failed to send message");
  }

  freespace_message response;
  ret = freespace_readMessage(ids[0], &response, 1000);
  if(ret != FREESPACE_SUCCESS) {
    if(ret == FREESPACE_ERROR_TIMEOUT) {
      ROS_ERROR_STREAM("failed to read IMU TIMEOUT");
    } else {
      ROS_ERROR_STREAM("failed to read IMU" << ret);
    }
  }
  if(response.messageType == FREESPACE_MESSAGE_DATAMODECONTROLV2RESPONSE ) {
    ROS_INFO_STREAM("got response" << response.dataModeControlV2Response.modeActual);
  } else {
    ROS_ERROR_STREAM("got invalid type " << response.messageType);
  }

  while (ros::ok())
  {
    ros::spinOnce();

    ret = freespace_readMessage(ids[0], &response, 1000);
    if(ret != FREESPACE_SUCCESS) {
      if(ret == FREESPACE_ERROR_TIMEOUT) {
        ROS_ERROR_STREAM("failed to read IMU TIMEOUT");
      } else {
        ROS_ERROR_STREAM("failed to read IMU" << ret);
      }
    }

    if (response.messageType != FREESPACE_MESSAGE_MOTIONENGINEOUTPUT) {
      ROS_ERROR_STREAM("error got unknown message type " << response.messageType);
    }

    MultiAxisSensor accel_msg;
    ret = freespace_util_getAcceleration(&response.motionEngineOutput, &accel_msg);
    if(ret != FREESPACE_SUCCESS) {
      ROS_ERROR_STREAM("failed to read acceleration " << ret);
    }
    Eigen::Vector3d accel_vec(accel_msg.x,-accel_msg.y,accel_msg.z);
    Eigen::Matrix3d T;
    T << cos(M_PI), -sin(M_PI), 0, sin(M_PI), cos(M_PI), 0, 0, 0, 1;
    Eigen::Vector3d rotated_accel = T * accel_vec;

    MultiAxisSensor orientation_msg;
    ret = freespace_util_getAngPos(&response.motionEngineOutput, &orientation_msg);
    if(ret != FREESPACE_SUCCESS) {
      ROS_ERROR_STREAM("failed to read angular position " << ret);
    }

    MultiAxisSensor angular_vel_msg;
    ret = freespace_util_getAngularVelocity(&response.motionEngineOutput, &angular_vel_msg);
    if(ret != FREESPACE_SUCCESS) {
      ROS_ERROR_STREAM("failed to read angular velocity " << ret);
    }
    T << 1, 0, 0, 0, cos(M_PI), -sin(M_PI), 0, sin(M_PI), cos(M_PI);
    Eigen::Vector3d angular_vel_vec(angular_vel_msg.x,angular_vel_msg.y,angular_vel_msg.z);
    Eigen::Vector3d rotated_angular_vel = T * angular_vel_vec;

    MultiAxisSensor magnetometer_msg;
    ret = freespace_util_getMagnetometer(&response.motionEngineOutput, &magnetometer_msg);
    if(ret != FREESPACE_SUCCESS) {
      ROS_ERROR_STREAM("failed to read raw magnetometer velocity " << ret);
    }

    // declare the message to be published
    sensor_msgs::Imu msg;
    msg.header.frame_id = "imu";
    msg.header.stamp = ros::Time::now();
    msg.header.seq = seq++;

    msg.linear_acceleration.x = rotated_accel[0];
    msg.linear_acceleration.y = rotated_accel[1];
    msg.linear_acceleration.z = rotated_accel[2];

    msg.angular_velocity.x = rotated_angular_vel[0];
    msg.angular_velocity.y = rotated_angular_vel[1];
    msg.angular_velocity.z = rotated_angular_vel[2];
    msg.angular_velocity_covariance = { 0.02, 1e-6, 1e-6, 1e-6, 0.02, 1e-6, 1e-6, 1e-6, 0.02 };

    tf::Quaternion pre_rotated(orientation_msg.x, orientation_msg.y, orientation_msg.z, orientation_msg.w);
    tf::Quaternion rotation = tf::createQuaternionFromRPY(0, 0, yaw_offset);

    double roll, pitch, yaw;
    tf::Matrix3x3(pre_rotated).getRPY(roll, pitch, yaw);
    std_msgs::Float64 raw_yaw_msg;
    raw_yaw_msg.data = yaw;
    raw_yaw_pub.publish(raw_yaw_msg);

    tf::Quaternion rotated = rotation * pre_rotated;


    geometry_msgs::Quaternion orientation;
    orientation.x = rotated.x();
    orientation.y = rotated.y();
    orientation.z = rotated.z();
    orientation.w = rotated.w();

    msg.orientation = orientation;
    msg.orientation_covariance = { 0.0025, 1e-6, 1e-6, 1e-6, 0.0025, 1e-6, 1e-6, 1e-6, 0.0025 };

    imu_pub.publish(msg);

    geometry_msgs::Vector3 raw_mag;
    raw_mag.x = magnetometer_msg.x;
    raw_mag.y = magnetometer_msg.y;
    raw_mag.z = magnetometer_msg.z;

    raw_mag_pub.publish(raw_mag);

  }
  return 0;
}
