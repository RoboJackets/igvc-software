// This node reads and publishes IMU data from the FSM-9 IMU.

#include <freespace/freespace.h>
#include <freespace/freespace_util.h>

#include <ros/publisher.h>
#include <ros/ros.h>

#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>

#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <igvc_utils/NodeUtils.hpp>

#include <vector>

struct IMUMessage
{
  MultiAxisSensor accel_msg;
  MultiAxisSensor accel_raw_msg;
  MultiAxisSensor orientation_msg;
  MultiAxisSensor angular_vel_msg;
  MultiAxisSensor magnetometer_msg;
};

class IMU
{
public:
  IMU();

private:
  using degrees = double;
  using radians = double;

  unsigned int seq;
  degrees m_heading_deviation_threshold{};
  radians m_yaw_offset{};
  Eigen::Matrix3d m_correction_matrix;
  int m_imu_id;

  std::vector<double> m_orientation_transform;
  std::vector<double> m_linear_accel_cov;
  std::vector<double> m_angular_vel_cov;
  std::vector<double> m_orientation_cov;

  ros::Publisher m_imu_no_gravity_pub;
  ros::Publisher m_imu_gravity_pub;
  ros::Publisher m_magnetic_field_pub;

  int initialize_imu(FreespaceDeviceId *ids);
  int read_imu(IMUMessage &message) const;
  void publish_imu_data(const IMUMessage &message) const;
};

IMU::IMU() : seq{ 0 }
{
  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  // Get matrix to correct IMU orientation.
  igvc::getParam(nh, "imu_orientation_correction", m_orientation_transform);

  igvc::getParam(nh, "linear_accel_cov", m_linear_accel_cov);
  igvc::getParam(nh, "angular_velocity_cov", m_angular_vel_cov);
  igvc::getParam(nh, "orientation_cov", m_orientation_cov);


  // Get threshold value for triggering a warning when the deviation between the filtered
  // heading and magnetometer heading becomes too large.
  igvc::getParam(nh, "heading_dev_thresh_deg", m_heading_deviation_threshold);

  // Get yaw offset for making yaw relative to East.
  igvc::getParam(nh, "yaw_offset", m_yaw_offset);

  // There should be 9 elements (3x3 matrix).
  if (m_orientation_transform.size() != 9)
  {
    ROS_ERROR_STREAM("IMU orientation transform matrix does not have 9 elements (3x3).");
    ros::shutdown();
  }
  if (m_linear_accel_cov.size() != 3)
  {
    ROS_ERROR_STREAM("Linear acceleration covariance must have 3 elements.");
    ros::shutdown();
  }
  if (m_angular_vel_cov.size() != 3)
  {
    ROS_ERROR_STREAM("Angular velocity covariance must have 3 elements.");
    ros::shutdown();
  }
  if (m_orientation_cov.size() != 3)
  {
    ROS_ERROR_STREAM("Orientation covariance must have 3 elements.");
    ros::shutdown();
  }

  m_correction_matrix = Eigen::Matrix3d(m_orientation_transform.data());

  m_imu_no_gravity_pub = nh.advertise<sensor_msgs::Imu>("/imu", 1000);
  m_imu_gravity_pub = nh.advertise<sensor_msgs::Imu>("/imu_raw", 1000);
  m_magnetic_field_pub = nh.advertise<sensor_msgs::MagneticField>("/imu/magnetic_field", 1000);

  // Initialize IMU.
  FreespaceDeviceId ids[1];
  int return_code = initialize_imu(ids);
  if (return_code != FREESPACE_SUCCESS)
  {
    ROS_ERROR_STREAM("Failed to initialize IMU (Return Code " << return_code << ")");
    ros::shutdown();
  }
  else
  {
    m_imu_id = ids[0];
  }

  while (ros::ok())
  {
    ros::spinOnce();
    IMUMessage message{};
    if (read_imu(message) != EXIT_FAILURE) {
      publish_imu_data(message);
    }
  }
}

/**
 * Reads message from IMU
 * @param[out] imu_message Variable for imu messages to be stored
 * @return EXIT_SUCCESS if read successfully, EXIT_FAILURE if a failure occured.
 */
int IMU::read_imu(IMUMessage &imu_message) const
{
  // Read the IMU message every loop. Timeout after 1000 milliseconds.
  freespace_message response{};
  int return_code = freespace_readMessage(m_imu_id, &response, 1000);
  if (return_code != FREESPACE_SUCCESS)
  {
    if (return_code == FREESPACE_ERROR_TIMEOUT)
    {
      ROS_ERROR_STREAM("failed to read IMU TIMEOUT");
    }
    else
    {
      ROS_ERROR_STREAM("failed to read IMU" << return_code);
    }
    return EXIT_FAILURE;
  }

  if (response.messageType != FREESPACE_MESSAGE_MOTIONENGINEOUTPUT)
  {
    ROS_ERROR_STREAM("IMU error: got unknown message type " << response.messageType);
    return EXIT_FAILURE;
  }

  return_code = freespace_util_getAccNoGravity(&response.motionEngineOutput, &imu_message.accel_msg);
  if (return_code != FREESPACE_SUCCESS)
  {
    ROS_ERROR_STREAM("failed to read acceleration no gravity." << return_code);
    return EXIT_FAILURE;
  }

  return_code = freespace_util_getAcceleration(&response.motionEngineOutput, &imu_message.accel_raw_msg);
  if (return_code != FREESPACE_SUCCESS)
  {
    ROS_ERROR_STREAM("failed to read acceleration." << return_code);
    return EXIT_FAILURE;
  }
  return_code = freespace_util_getAngPos(&response.motionEngineOutput, &imu_message.orientation_msg);
  if (return_code != FREESPACE_SUCCESS)
  {
    ROS_ERROR_STREAM("failed to read angular position " << return_code);
    return EXIT_FAILURE;
  }
  return_code = freespace_util_getAngularVelocity(&response.motionEngineOutput, &imu_message.angular_vel_msg);
  if (return_code != FREESPACE_SUCCESS)
  {
    ROS_ERROR_STREAM("failed to read angular velocity " << return_code);
    return EXIT_FAILURE;
  }

  return_code = freespace_util_getMagnetometer(&response.motionEngineOutput, &imu_message.magnetometer_msg);
  if (return_code != FREESPACE_SUCCESS)
  {
    ROS_ERROR_STREAM("failed to read raw magnetometer values" << return_code);
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

/**
 * Publishes data from imu
 * @param[in] imu_message variable containing data to be used for publishing
 */
void IMU::publish_imu_data(const IMUMessage &imu_message) const
{
  // ============================= Transform message frames using m_correction_matrix ==================================
  Eigen::Vector3d accel_raw_vec(imu_message.accel_raw_msg.x, imu_message.accel_raw_msg.y, imu_message.accel_raw_msg.z);
  accel_raw_vec = m_correction_matrix * accel_raw_vec;

  Eigen::Vector3d accel_vec(imu_message.accel_msg.x, imu_message.accel_msg.y, imu_message.accel_msg.z);
  accel_vec = m_correction_matrix * accel_vec;

  tf::Quaternion quaternion_raw(imu_message.orientation_msg.x, imu_message.orientation_msg.y,
                                imu_message.orientation_msg.z, imu_message.orientation_msg.w);
  double roll, pitch, yaw;
  tf::Matrix3x3(quaternion_raw).getRPY(roll, pitch, yaw);
  yaw += m_yaw_offset;

  // Get gyroscope measurements.

  Eigen::Vector3d angular_vel_vec(imu_message.angular_vel_msg.x, imu_message.angular_vel_msg.y, imu_message.angular_vel_msg.z);
  angular_vel_vec = m_correction_matrix * angular_vel_vec;

  // Calculate heading with magnetometer reading.
  double yaw_mag = atan2(imu_message.magnetometer_msg.y, imu_message.magnetometer_msg.x) + m_yaw_offset;

  if (fabs(yaw - yaw_mag) > m_heading_deviation_threshold * M_PI / 180.0)
  {
    ROS_WARN("Magnetometer heading and filtered yaw measurement disagree by > %f degrees.", m_heading_deviation_threshold);
  }

  // =================================== Publish transformed measurements ==============================================
  static unsigned int sequence = 0;
  sensor_msgs::Imu msg{};
  msg.header.frame_id = "imu";
  msg.header.stamp = ros::Time::now();
  msg.header.seq = sequence++;

  msg.linear_acceleration.x = accel_vec[0];
  msg.linear_acceleration.y = accel_vec[1];
  msg.linear_acceleration.z = accel_vec[2];
  msg.linear_acceleration_covariance = { m_linear_accel_cov[0], 1e-6, 1e-6,
                                         1e-6, m_linear_accel_cov[1], 1e-6,
                                         1e-6, 1e-6, m_linear_accel_cov[2]};

  msg.angular_velocity.x = angular_vel_vec[0];
  msg.angular_velocity.y = angular_vel_vec[1];
  msg.angular_velocity.z = angular_vel_vec[2];
  msg.angular_velocity_covariance = { m_angular_vel_cov[0], 1e-6, 1e-6,
                                      1e-6, m_angular_vel_cov[1], 1e-6,
                                      1e-6, 1e-6, m_angular_vel_cov[2]};

  tf::Quaternion quaternion_mag;
  quaternion_mag.setRPY(0, 0, yaw_mag);

  geometry_msgs::Quaternion orientation{};
  orientation.x = quaternion_mag.x();
  orientation.y = quaternion_mag.y();
  orientation.z = quaternion_mag.z();
  orientation.w = quaternion_mag.w();

  // TODO: Tune and parameterize.
  msg.orientation = orientation;
  msg.orientation_covariance = { m_orientation_cov[0], 1e-6, 1e-6,
                                 1e-6, m_orientation_cov[1], 1e-6,
                                 1e-6, 1e-6, m_orientation_cov[2]};

  m_imu_no_gravity_pub.publish(msg);

  // Publish raw sensor messages.
  sensor_msgs::Imu msg_imu_gravity{};
  msg_imu_gravity.header.frame_id = "imu";
  msg_imu_gravity.header.stamp = msg.header.stamp;
  msg_imu_gravity.header.seq = seq;

  msg_imu_gravity.linear_acceleration.x = accel_raw_vec[0];
  msg_imu_gravity.linear_acceleration.y = accel_raw_vec[1];
  msg_imu_gravity.linear_acceleration.z = accel_raw_vec[2];
  msg.linear_acceleration_covariance = { m_linear_accel_cov[0], 1e-6, 1e-6,
                                         1e-6, m_linear_accel_cov[1], 1e-6,
                                         1e-6, 1e-6, m_linear_accel_cov[2]};

  msg_imu_gravity.angular_velocity.x = angular_vel_vec[0];
  msg_imu_gravity.angular_velocity.y = angular_vel_vec[1];
  msg_imu_gravity.angular_velocity.z = angular_vel_vec[2];
  msg.angular_velocity_covariance = { m_angular_vel_cov[0], 1e-6, 1e-6,
                                      1e-6, m_angular_vel_cov[1], 1e-6,
                                      1e-6, 1e-6, m_angular_vel_cov[2]};

  msg_imu_gravity.orientation = orientation;
  msg.orientation_covariance = { m_orientation_cov[0], 1e-6, 1e-6,
                                 1e-6, m_orientation_cov[1], 1e-6,
                                 1e-6, 1e-6, m_orientation_cov[2]};

  m_imu_gravity_pub.publish(msg_imu_gravity);

  // Publish Raw Magnetic Field values. No information about covariances, so left as 0
  sensor_msgs::MagneticField msg_magnetic_field{};
  msg_magnetic_field.header.frame_id = "imu";
  msg_magnetic_field.header.stamp = msg.header.stamp;
  msg_magnetic_field.header.seq = seq;

  msg_magnetic_field.magnetic_field.x = imu_message.magnetometer_msg.x;
  msg_magnetic_field.magnetic_field.y = imu_message.magnetometer_msg.y;
  msg_magnetic_field.magnetic_field.z = imu_message.magnetometer_msg.z;

  m_magnetic_field_pub.publish(msg_magnetic_field);
}

/** A function for initializing a single FSM-9 IMU. This function sets up the IMU
 * such that it returns expected measurements (ex. linear acceleration, angular velocity, etc.).
 * \param ids an array for storing the identified IMU device ID.
 */
int IMU::initialize_imu(FreespaceDeviceId ids[])
{
  int return_code = freespace_init();
  if (return_code != FREESPACE_SUCCESS)
  {
    ROS_ERROR_STREAM("Failed to init IMU");
    return return_code;
  }

  // Initialize freespace device (we expect only 1).
  int num_found = 0;
  return_code = freespace_getDeviceList(ids, 1, &num_found);
  if (num_found <= 0)
  {
    ROS_ERROR_STREAM("Failed to connect to IMU, found no devices");
    return return_code;
  }
  ROS_INFO_STREAM("Found " << num_found << " devices.");

  return_code = freespace_openDevice(ids[0]);
  if (return_code != FREESPACE_SUCCESS)
  {
    ROS_ERROR_STREAM("failure to connect to device " << return_code);
    return return_code;
  }

  freespace_message request{};
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
  request.dataModeControlV2Request.ff5 = 1;              // Enable Temperature
  request.dataModeControlV2Request.ff6 = 1;              // Enable Angular Position (WXYZ)
  request.dataModeControlV2Request.ff7 = 0;              // Nothing
  return_code = freespace_sendMessage(ids[0], &request);
  if (return_code != FREESPACE_SUCCESS)
  {
    ROS_ERROR_STREAM("failed to send message");
  }
  return return_code;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu");
  IMU imu;
}
