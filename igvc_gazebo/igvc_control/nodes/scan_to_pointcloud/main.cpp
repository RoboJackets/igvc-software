#include <laser_geometry/laser_geometry.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher _pointcloud_pub;
laser_geometry::LaserProjection projection;

double min_dist, neighbor_dist;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  sensor_msgs::LaserScan scanData = *msg;

  auto rangeIsValid = [&scanData](float range) {
    return !std::isnan(range) && range > scanData.range_min && range < scanData.range_max;
  };

  for (unsigned int i = 0; i < scanData.ranges.size(); i++)
  {
    float& range = scanData.ranges[i];
    if (range > scanData.range_max || range < scanData.range_min)
      continue;
    // Too close
    if (range < min_dist)
      range = scanData.range_max + 1;
    // No valid neighbors
    else if ((i == 0 || !rangeIsValid(scanData.ranges[i - 1])) &&
             (i == (scanData.ranges.size() - 1) || !rangeIsValid(scanData.ranges[i + 1])))
      range = scanData.range_max + 1;
    // No close neighbors
    else if ((i == 0 || abs(scanData.ranges[i - 1] - range) > neighbor_dist) &&
             (i == (scanData.ranges.size() - 1) || abs(scanData.ranges[i + 1] - range) > neighbor_dist))
      range = scanData.range_max + 1;
  }

  sensor_msgs::PointCloud2 cloud;
  projection.projectLaser(scanData, cloud);
  cloud.header.frame_id = "/lidar";

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_for_pub(new pcl::PointCloud<pcl::PointXYZ>());
  fromROSMsg(cloud, *cloud_for_pub);
  _pointcloud_pub.publish(*cloud_for_pub);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_to_pointcloud");

  ros::NodeHandle pNh("~");

  pNh.getParam("min_dist", min_dist);
  pNh.getParam("neighbor_dist", neighbor_dist);

  ros::NodeHandle nh;

  _pointcloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/pc2", 1);

  ros::Subscriber scan_sub = nh.subscribe("/scan", 1, scanCallback);

  ros::spin();
}
