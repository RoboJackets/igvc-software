#include <ros/ros.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <parameter_assertions/assertions.h>
#include <tf/transform_datatypes.h>
#include <igvc_utils/NodeUtils.hpp>

ros::Publisher g_rpy_pub;

void convertAndPublish(const geometry_msgs::Quaternion& quat, const ros::Time& stamp)
{
  geometry_msgs::Vector3Stamped msg;
  msg.header.stamp = stamp;

  tf::Quaternion tf_quat;
  tf::quaternionMsgToTF(quat, tf_quat);

  double r, p, y;
  tf::Matrix3x3(tf_quat).getRPY(r, p, y);

  msg.vector.x = r;
  msg.vector.y = p;
  msg.vector.z = y;

  g_rpy_pub.publish(msg);
}

void imu_callback(const sensor_msgs::Imu& imu)
{
  convertAndPublish(imu.orientation, imu.header.stamp);
}

void odom_callback(const nav_msgs::Odometry& odom)
{
  convertAndPublish(odom.pose.pose.orientation, odom.header.stamp);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "quaternion_to_rpy");

  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  std::string quaternion_topic;
  std::string rpy_topic;
  std::string message_type;

  std::string imu_quaternion = "imu";
  std::string odom_quaternion = "odometry";

  assertions::getParam(pNh, "topics/quaternion", quaternion_topic);
  assertions::getParam(pNh, "topics/rpy", rpy_topic);
  assertions::param(pNh, "message_type", message_type, imu_quaternion);

  g_rpy_pub = nh.advertise<geometry_msgs::Vector3Stamped>(rpy_topic, 1);

  ros::Subscriber sub;
  if (message_type == imu_quaternion)
  {
    ROS_INFO_STREAM("got imu message_type. Subscribing to " << quaternion_topic);
    sub = nh.subscribe(quaternion_topic, 1, imu_callback);
  }
  else if (message_type == odom_quaternion)
  {
    ROS_INFO_STREAM("got odom message_type. Subscribing to " << quaternion_topic);
    sub = nh.subscribe(quaternion_topic, 1, odom_callback);
  }
  else
  {
    ROS_ERROR_STREAM("message_type not 'imu' or 'odometry'. Exiting...");
    exit(1);
  }

  ros::spin();
}
