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

class scan_to_pointcloud {
private:
  ros::Publisher _pointcloud_pub;
  ros::Subscriber _pointcloud_sub;

  double min_dist;
  double neighbor_dist;
  double offset;

public:
  explicit scan_to_pointcloud(ros::NodeHandle node_handle,
                              ros::NodeHandle private_node_handle);
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
  ~scan_to_pointcloud();
};

#endif