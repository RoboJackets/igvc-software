#include <ros/ros.h>
#include <ros/publisher.h>
#include <cmath>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace pcl;
using namespace ros;
using namespace std;

Publisher _pointcloud_pub;
laser_geometry::LaserProjection projection;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

    sensor_msgs::PointCloud2 cloud;
    projection.projectLaser(*msg, cloud);
    cloud.header.frame_id = "/lidar";

    PointCloud<PointXYZ>::Ptr cloud_for_pub(new PointCloud<PointXYZ>());
    fromROSMsg(cloud, *cloud_for_pub);

    {
        KdTreeFLANN<PointXYZ> kdtree;
        kdtree.setInputCloud(cloud_for_pub);
        vector<int> pointIndeces;
        vector<float> squaredDistances;
        kdtree.radiusSearch(PointXYZ(0,0,0), 0.5, pointIndeces, squaredDistances);
        for(auto idx : pointIndeces)
            cloud_for_pub->erase(cloud_for_pub->begin() + idx);
    }

    _pointcloud_pub.publish(*cloud_for_pub);
}

int main(int argc, char** argv)
{
    init(argc, argv, "scan_to_pointcloud");

    NodeHandle nh;

    _pointcloud_pub = nh.advertise<PointCloud<PointXYZ> >("/scan/pointcloud", 1);

    Subscriber scan_sub = nh.subscribe("/scan", 1, scanCallback);

    spin();
}
