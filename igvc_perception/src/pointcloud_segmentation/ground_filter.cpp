#include "ground_filter.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

typedef pcl::PointCloud<pcl::PointXYZ> PC;

GroundFilterNode::GroundFilterNode()
{
  private_nh_ = ros::NodeHandle("~");
  std::string input_topic, filtered_topic;
  //private_nh_.getParam("/ground_filter_node/topic/input", input_topic);
  //private_nh_.getParam("/ground_filter_node/topic/filtered", filtered_topic);
  raw_pts_sub_ = private_nh_.subscribe("/lidar/transformed", 1, &GroundFilterNode::groundFilterCallback, this);
  ground_filter_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("ground_segmentation", 1);
};

void GroundFilterNode::groundFilterCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  PC::Ptr cloud(new PC);
  PC::Ptr cloud_filtered(new PC);
  pcl::fromROSMsg(*cloud_msg, *cloud);
  
  float x, y, z_min, z_max; std::string frame_id;
  private_nh_.getParam("/ground_filter_node/range/x", x);
  private_nh_.getParam("/ground_filter_node/range/y", y);
  private_nh_.getParam("/ground_filter_node/range/height_min", z_min);
  private_nh_.getParam("/ground_filter_node/range/height_max", z_max);
  private_nh_.getParam("/ground_filter_node/frame_id", frame_id);

  for (unsigned int i = 0; i < cloud->points.size(); i++)
  {
    pcl::PointXYZ p = cloud->points[i];
    bool within_thresholds = -x <= p.x && p.x <= x && -y <= p.y && p.y <= y && z_min <= p.z && p.z <= z_max;
    if (within_thresholds)
    {
      cloud_filtered->push_back(p);
    }
  }

  sensor_msgs::PointCloud2 output_msg;
  pcl::toROSMsg(*cloud_filtered, output_msg);
  output_msg.header.frame_id = frame_id;
  output_msg.header.stamp = ros::Time::now();
  ground_filter_pub_.publish(output_msg);
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ground_filter");
  GroundFilterNode ground_filter = GroundFilterNode();
  ros::spin();
}