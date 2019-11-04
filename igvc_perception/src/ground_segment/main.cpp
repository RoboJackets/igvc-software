#include <ros/ros.h>
#include <parameter_assertions/assertions.h>
#include <math.h>
#include "GroundSegmenter.h"

ros::Publisher g_ground_pub;
ros::Publisher g_nonground_pub;
ros::Subscriber g_velodyne_sub;
std::string g_ground_topic;
std::string g_nonground_topic;
std::string g_velodyne_topic;
int g_num_segments;
int g_bin_width;

void velodyneCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg)
{ 
  GroundSegmenter segmenter = GroundSegmenter(msg->points, g_num_segments, g_bin_width);
  segmenter.processSegments();
  segmenter.getLinesFromSegments();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ground_segment");

  ros::NodeHandle n;
  ros::NodeHandle pNh("~");

  assertions::getParam(pNh, "ground_topic", g_ground_topic);
  assertions::getParam(pNh, "nonground_topic", g_nonground_topic);
  assertions::getParam(pNh, "velodyne_topic", g_velodyne_topic);
  assertions::getParam(pNh, "num_segments", g_num_segments);
  assertions::getParam(pNh, "bin_width", g_bin_width);

  g_ground_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ>>(g_ground_topic, 1);
  g_nonground_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ>>(g_nonground_topic, 1);

  g_velodyne_sub = n.subscribe(g_velodyne_topic, 1, velodyneCallback);

  ros::spin();

}