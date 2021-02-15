// Magnetometer.h
#ifndef Magnetometer_H
#define Magnetometer_H

#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>

class Magnetometer 
{

public:
	Magnetometer::Magnetometer();
private:
	ros::Publisher mag_field_pub;
	static double mag_field_covar;
	void magCallback(const geometry_msgs::Vector3Stamped& msg);
};
#endif