#include <ros/ros.h>
#include <stdlib.h>
#include <ros/publisher.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <set>

using namespace std;
using namespace pcl;
using namespace ros;

PointCloud<PointXYZ>::Ptr map_cloud;
Publisher _pointcloud_pub;
tf::TransformListener *tf_listener;
set<string> frames_seen;

void filterOutDuplicates(PointCloud<PointXYZ>::ConstPtr cloud)
{
    VoxelGrid<PointXYZ> filter;
    filter.setInputCloud(cloud);
    filter.setLeafSize(0.01f, 0.01f, 0.01f);
    filter.filter(*map_cloud);
}

void nodeCallback(const PointCloud<PointXYZ>::ConstPtr &msg)
{
    PointCloud<PointXYZ> transformed;
    tf::StampedTransform transform;
    if(frames_seen.find(msg->header.frame_id) != frames_seen.end())
    {
        frames_seen.insert(msg->header.frame_id);
        tf_listener->waitForTransform("/map", msg->header.frame_id, Time(0), Duration(5));
    }
    tf_listener->lookupTransform("/map", msg->header.frame_id, Time(0), transform);
    pcl_ros::transformPointCloud(*msg, transformed, transform);

    *map_cloud += transformed;

    filterOutDuplicates(map_cloud);

    _pointcloud_pub.publish(map_cloud);
}

int main(int argc, char** argv)
{
    map_cloud = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>());

    init(argc, argv, "mapper");

    NodeHandle nh;
    tf_listener = new tf::TransformListener();

    string topics;

    list<Subscriber> subs;

    NodeHandle pNh("~");

    if(!pNh.hasParam("topics"))
        ROS_WARN_STREAM("No topics specified for mapper. No map will be generated.");

    pNh.getParam("topics", topics);

    if(topics.empty())
        ROS_WARN_STREAM("No topics specified for mapper. No map will be generated.");

    istringstream iss(topics);
    vector<string> tokens { istream_iterator<string>(iss), istream_iterator<string>() };

    for(auto topic : tokens)
    {
        ROS_INFO_STREAM("Mapper subscribing to " << topic);
        subs.push_back(nh.subscribe(topic, 1, nodeCallback));
    }

    map_cloud->header.frame_id = "/map";

    _pointcloud_pub = nh.advertise<PointCloud<PointXYZ> >("/map", 1);

    spin();
}
