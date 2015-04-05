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
    _pointcloud_pub.publish(cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar");

    ros::NodeHandle nh;

    _pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud/lidar", 1);

    ros::Subscriber scan_sub = nh.subscribe("/scan", 1, scanCallback);

    ros::spin();
}

