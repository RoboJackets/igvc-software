#include <ros/ros.h>
#include <ros/publisher.h>
#include <cmath>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

ros::Publisher _pointcloud_pub;
laser_geometry::LaserProjection proj;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

    sensor_msgs::PointCloud2 cloud;
    proj.projectLaser(*msg, cloud);
    cloud.header.frame_id = "/lidar";

    pcl::PointCloud<pcl::PointXYZ> cloud_for_pub;
    pcl::fromROSMsg(cloud, cloud_for_pub);

    _pointcloud_pub.publish(cloud_for_pub);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan_to_pointcloud");

    ros::NodeHandle nh;

    _pointcloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/scan/pointcloud", 1);

    ros::Subscriber scan_sub = nh.subscribe("/scan", 1, scanCallback);

    ros::spin();
}
