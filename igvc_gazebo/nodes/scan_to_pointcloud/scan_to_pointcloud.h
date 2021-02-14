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

class ScanToPointcloud
{
public:
  ScanToPointcloud();

private:
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  ros::NodeHandle nh;
  ros::Publisher _pointcloud_pub;
  laser_geometry::LaserProjection projection;

  double min_dist, neighbor_dist;
  double offset;
};
#endif