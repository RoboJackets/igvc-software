#include <laser_geometry/laser_geometry.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <igvc_utils/NodeUtils.hpp>

ros::Publisher _pointcloud_pub;
laser_geometry::LaserProjection projection;

double min_dist, neighbor_dist;

void scanCallback(const sensor_msgs::PointCloud& msg)
{
  sensor_msgs::PointCloud2 cloud;
  sensor_msgs::convertPointCloudToPointCloud2(msg, cloud);
  pcl::PointCloud<pcl::PointXYZ> cloud_for_pub;
  fromROSMsg(cloud, cloud_for_pub);
  _pointcloud_pub.publish(cloud_for_pub);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_to_pointcloud");

  ros::NodeHandle pNh("~");

  igvc::getParam(pNh, "min_dist", min_dist);
  igvc::getParam(pNh, "neighbor_dist", neighbor_dist);

  ros::NodeHandle nh;

  _pointcloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/scan/pointcloud", 1);

  ros::Subscriber scan_sub = nh.subscribe("/scan", 1, scanCallback);

  ros::spin();
}
