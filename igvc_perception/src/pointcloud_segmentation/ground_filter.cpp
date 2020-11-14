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
    raw_pts_sub_ = private_nh_.subscribe("/velodyne_points", 1, &GroundFilterNode::groundFilterCallback, this);
    ground_filter_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2> ("ground_segmentation", 1);
};


void GroundFilterNode::groundFilterCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  PC::Ptr cloud (new PC);
  PC::Ptr cloud_filtered (new PC);
  pcl::fromROSMsg(*cloud_msg, *cloud);
  const auto& x = 5.0;
  const auto& y = 5.0;
  const auto& z = 2.0;
  
  for (unsigned int i = 0; i < cloud->points.size(); i++) {
      pcl::PointXYZ p = cloud->points[i];
      bool within_thresholds = -x <= p.x && p.x <= x && -y <= p.y && p.y <= y && 0.4 <= p.z && p.z <= z;
      if (within_thresholds) {
          cloud_filtered->push_back(p);
      }
  }

  sensor_msgs::PointCloud2 output_msg;
  pcl::toROSMsg(*cloud_filtered, output_msg);
  output_msg.header.frame_id = "base_link";
  output_msg.header.stamp = ros::Time::now();
  ground_filter_pub_.publish (output_msg);
};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "ground_filter");
  GroundFilterNode ground_filter = GroundFilterNode();
  ros::spin();
}