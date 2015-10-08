#include <ros/ros.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "hello_world");
	ros::NodeHandle nh;
	ROS_INFO_STREAM("Hello World!");
}
