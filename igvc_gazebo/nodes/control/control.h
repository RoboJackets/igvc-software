#ifndef Control_h
#define Control_h

#include <igvc_msgs/velocity_pair.h>
#include <sensor_msgs/JointState.h>


class Control
{

public:
	Control();
private:
	double speed_set_point_left;
	double speed_set_point_right;
	double speed_measured_left;
	double speed_measured_right;
	double wheel_radius;
	void speedCallback(const igvc_msgs::velocity_pair::ConstPtr &msg);
	void jointStateCallback(const sensor_msgs::JointStateConstPtr &msg);
	
};
#endif