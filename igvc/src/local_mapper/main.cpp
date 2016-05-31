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
#include <climits>
#include <pcl/common/geometry.h>
#include <map>
#include <functional>

using namespace std;
using namespace pcl;
using namespace ros;

PointCloud<PointXYZ>::Ptr map_cloud;
Publisher _pointcloud_pub;
tf::TransformListener *tf_listener;
set<string> frames_seen;

// Map of topic : cloud
map<string, PointCloud<PointXYZ> > frames;

void publish_map()
{
    map_cloud->clear();

    for(auto cloud : frames)
        *map_cloud += cloud.second;

    _pointcloud_pub.publish(map_cloud);
}

void frame_callback(const PointCloud<PointXYZ>::ConstPtr &msg, const string &topic)
{
    PointCloud<PointXYZ> transformed;
    tf::StampedTransform transform;
    if(frames_seen.find(msg->header.frame_id) == frames_seen.end())
    {
        frames_seen.insert(msg->header.frame_id);
        tf_listener->waitForTransform("/map", msg->header.frame_id, Time(0), Duration(5));
    }
    tf_listener->lookupTransform("/map", msg->header.frame_id, Time(0), transform);
    pcl_ros::transformPointCloud(*msg, transformed, transform);
    for (auto point : transformed) {
        point.z = 0;
    }

    frames[topic] = transformed;

    publish_map();
}

int main(int argc, char** argv)
{
    map_cloud = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>());

    init(argc, argv, "local_mapper");

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
        subs.push_back(nh.subscribe<PointCloud<PointXYZ> >(topic, 1, boost::bind(frame_callback, _1, topic)));
    }

    map_cloud->header.frame_id = "/map";

    _pointcloud_pub = nh.advertise<PointCloud<PointXYZ> >("/map", 1);

    spin();
}
