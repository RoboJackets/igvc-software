#include <ros/ros.h>
#include <stdlib.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/publisher.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/point_cloud_conversion.h>

sensor_msgs::PointCloud2 cloud;
ros::Publisher _pointcloud_pub;
tf::TransformListener *tf_listener;

void nodeCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    sensor_msgs::PointCloud transformed;
    sensor_msgs::convertPointCloud2ToPointCloud(*msg, transformed);
    tf_listener->transformPointCloud("/mapper",transformed,transformed);

    sensor_msgs::PointCloud2 transformed2;
    sensor_msgs::convertPointCloudToPointCloud2(transformed, transformed2);
    pcl::concatenatePointCloud(transformed2, cloud, cloud);

    _pointcloud_pub.publish(cloud);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "map");

    ros::NodeHandle nh;
    tf_listener = new tf::TransformListener();

    std::string topics;
    std::string delimiter = " ";

    std::list<ros::Subscriber> subs;

    ros::NodeHandle pNh("~");

    if(!pNh.hasParam("topics"))
        ROS_ERROR_STREAM("no param");

    pNh.getParam("topics", topics);

    int pos;
    while((pos = topics.find(delimiter)) != std::string::npos)
    {
        std::string token = topics.substr(0, pos);
        ROS_ERROR_STREAM(token);
        topics.erase(0,pos+delimiter.length());

        subs.push_back(nh.subscribe(token, 1, nodeCallback));
    }
    ROS_ERROR_STREAM(topics);
    subs.push_back(nh.subscribe(topics, 1, nodeCallback));

    cloud.header.frame_id = "/mapper";

    _pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud/map", 1);

	ros::spin();
}
