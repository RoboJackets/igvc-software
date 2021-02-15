#include "GroundTruth.h"

#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <parameter_assertions/assertions.h>
#include <robot_localization/navsat_conversions.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <mutex>

int main (int argc, char** argv)
{
	ros::init(argc, argv, "ground_truth_republisher");
	GroundTruth gt;
	ros::spin();
}