#include <ros/ros.h>
#include <stdlib.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/publisher.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>

using namespace std;

sensor_msgs::PointCloud2 cloud;
ros::Publisher _pointcloud_pub;
tf::TransformListener *tf_listener;

void nodeCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    sensor_msgs::PointCloud transformed;
    sensor_msgs::convertPointCloud2ToPointCloud(*msg, transformed);
    transformed.header.stamp = ros::Time(0);
    tf_listener->transformPointCloud("/map",transformed,transformed);

    sensor_msgs::PointCloud2 transformed2;
    sensor_msgs::convertPointCloudToPointCloud2(transformed, transformed2);
    sensor_msgs::PointCloud2 output;
    pcl::concatenatePointCloud(transformed2, cloud, output);
    cloud = output;

//    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//    sor.setInputCloud(cloud);
//    sor.setLeafSize (0.01f, 0.01f, 0.01f);
//    sor.filter(*cloud);

    _pointcloud_pub.publish(cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mapper");

    ros::NodeHandle nh;
    tf_listener = new tf::TransformListener();

    std::string topics;
    std::string delimiter = " ";

    std::list<ros::Subscriber> subs;

    ros::NodeHandle pNh("~");

    if(!pNh.hasParam("topics"))
        ROS_WARN_STREAM("No topics specified for mapper. No map will be generated.");

    pNh.getParam("topics", topics);

    if(topics.empty())
        ROS_WARN_STREAM("No topics specified for mapper. No map will be generated.");

    istringstream iss(topics);
    vector<string> tokens { istream_iterator<string>{iss},
                            istream_iterator<string>{} };
    for(auto topic : tokens)
    {
        ROS_INFO_STREAM("Mapper subscribing to " << topic);
        subs.push_back(nh.subscribe(topic, 1, nodeCallback));
    }

    cloud.header.frame_id = "/map";

    _pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/map", 1);

    tf_listener->waitForTransform("/map", "/lidar", ros::Time(0), ros::Duration(5));
	//Probably want to wait for other transforms that mapper will need like camera, camera_left, camera_right

	ros::spin();
}
