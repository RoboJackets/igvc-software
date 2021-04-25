// scan_to_pointcloud.h
#ifndef SCAN_TO_POINTCLOUD_H
#define SCAN_TO_POINTCLOUD_H

#include <laser_geometry/laser_geometry.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

double min_dist, neighbor_dist;
double offset;

laser_geometry::LaserProjection projection;
ros::Publisher _pointcloud_pub;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

class ScanToPointcloud
{
	public:
		ScanToPointcloud();

		ros::NodeHandle nh;		

		


};
#endif