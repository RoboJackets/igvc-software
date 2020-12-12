#include "Magnetometer.h"

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/MagneticField.h>

Magnetometer::Magnetometer()
{
	ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    std::string sub_topic = pnh.param("mag_sub_topic", std::string("/magnetometer/vector"));
    std::string pub_topic = pnh.param("mag_pub_topic", std::string("/magnetometer_mag"));
    g_mag_field_covar = pnh.param("mag_field_variance", 1e-6);
    g_mag_field_pub = nh.advertise<sensor_msgs::MagneticField>(pub_topic, 10);
    ros::Subscriber scan_sub = nh.subscribe(sub_topic, 1, magCallback);
}

void magCallback(const geometry_msgs::Vector3Stamped& msg)
{
    sensor_msgs::MagneticField magnet_msg;
    magnet_msg.header.seq = msg.header.seq;
    magnet_msg.header.stamp = msg.header.stamp;
    magnet_msg.header.frame_id = msg.header.frame_id;
    magnet_msg.magnetic_field.x = msg.vector.x;
    magnet_msg.magnetic_field.y = msg.vector.y;
    magnet_msg.magnetic_field.z = msg.vector.z;
    magnet_msg.magnetic_field_covariance = { g_mag_field_covar, 0, 0, 0, g_mag_field_covar, 0, 0, 0, g_mag_field_covar };
    g_mag_field_pub.publish(magnet_msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mag_republisher");
	Magnetometer m;
	ros::spin();
}
