#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <boost/array.hpp>

ros::Publisher pub;

double x;
double y;
double z;

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
  sensor_msgs::Imu result;
  result.header.frame_id = msg->header.frame_id;
  result.header.stamp = msg->header.stamp;
  result.header.seq = msg->header.seq;
  result.linear_acceleration_covariance = msg->linear_acceleration_covariance;
  result.angular_velocity_covariance = msg->angular_velocity_covariance;
  result.orientation_covariance = msg->orientation_covariance;

  result.linear_acceleration_covariance[0] = x;
  result.linear_acceleration_covariance[4] = y;
  result.linear_acceleration_covariance[8] = z;
  pub.publish(result);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_remap");

  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  pNh.param("x", x, 0.04);
  pNh.param("y", y, 0.04);
  pNh.param("z", z, 0.04);

  pub = nh.advertise<sensor_msgs::Imu>("/imu", 1000);

  ros::Subscriber sub = nh.subscribe("/imu_old", 1000, imu_callback);

  ros::spin();

  return 0;

}
