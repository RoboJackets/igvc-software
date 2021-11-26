#ifndef SCAN_TO_POINTCLOUD
#define SCAN_TO_POINTCLOUD

#include <laser_geometry/laser_geometry.h>
#include <parameter_assertions/assertions.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>

class scan_to_pointcloud
{
private:
  ros::Publisher _pointcloud_pub;
  ros::Subscriber _pointcloud_sub;
  ros::NodeHandle node_handle;  // look into private node handles

  laser_geometry::LaserProjection projection;  // probably look into what this does
                                               // it's only used once lmao

  double min_dist;
  double neighbor_dist;
  double offset;

public:
  scan_to_pointcloud();
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
};

#endif