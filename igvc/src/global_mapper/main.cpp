#include <ros/ros.h>
#include <stdlib.h>
#include <string>
#include <ros/publisher.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud;
ros::Publisher _pointcloud_pub;

void publish_map()
{
}

void frame_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg, const std::string &topic)
{
}

int main(int argc, char** argv)
{
    map_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

    ros::init(argc, argv, "global_mapper");

    ros::NodeHandle nh;

    std::string topics;

    std::list<ros::Subscriber> subs;

    ros::NodeHandle pNh("~");

    if(!pNh.hasParam("topics"))
        ROS_WARN_STREAM("No topics specified for mapper. No map will be generated.");

    pNh.getParam("topics", topics);

    if(topics.empty())
        ROS_WARN_STREAM("No topics specified for mapper. No map will be generated.");

    std::istringstream iss(topics);
    std::vector<std::string> tokens { std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>() };

    for(auto topic : tokens)
    {
        ROS_INFO_STREAM("Mapper subscribing to " << topic);
        subs.push_back(nh.subscribe<pcl::PointCloud<pcl::PointXYZ> >(topic, 1, boost::bind(frame_callback, _1, topic)));
    }

    map_cloud->header.frame_id = "/map";

    _pointcloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/map", 1);

    ros::spin();
}