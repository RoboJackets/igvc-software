#include <parameter_assertions/assertions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pointcloud_filter/pointcloud_filter.h>
#include <sensor_msgs/PointCloud2.h>

namespace pointcloud_filter
{
PointcloudFilter::PointcloudFilter(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
  : nh_{ nh }
  , private_nh_{ private_nh, "pointcloud_filter" }
  , config_{ private_nh_ }
  , buffer_{}
  , listener_{ buffer_ }
  , back_filter_{ private_nh_ }
  , radius_filter_{ private_nh_ }
  , tf_transform_filter_{ &buffer_ }
  , ground_filter_{ private_nh_ }
  , raycast_filter_{ private_nh_ }
  , fast_segment_filter_{ private_nh_ }
{
  setupPubSub();
}

void PointcloudFilter::setupPubSub()
{
  raw_pointcloud_sub_ = nh_.subscribe(config_.topic_input, 1, &PointcloudFilter::pointcloudCallback, this);

  transformed_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(config_.topic_transformed, 1);
  occupied_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(config_.topic_occupied, 1);
  free_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(config_.topic_free, 1);

  filtered_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(config_.topic_filtered, 1);
}

void PointcloudFilter::pointcloudCallback(const PointCloud::ConstPtr& raw_pointcloud)
{
  Bundle bundle{ raw_pointcloud };

  radius_filter_.filter(bundle);

  back_filter_.filter(bundle);

  ground_filter_.filter(bundle); // remove ground plane from original point cloud

  filtered_pointcloud_pub_.publish(bundle.pointcloud);

  fast_segment_filter_.filter(bundle);

  std::string base_frame = config_.base_frame;
  std::string lidar_frame = raw_pointcloud->header.frame_id;
  ros::Duration timeout{ config_.timeout_duration };

  tf_transform_filter_.transform(*bundle.pointcloud, *bundle.pointcloud, base_frame, timeout);

  tf_transform_filter_.transform(*bundle.occupied_pointcloud, *bundle.occupied_pointcloud, lidar_frame, timeout);
  raycast_filter_.filter(bundle);

  transformed_pointcloud_pub_.publish(bundle.pointcloud);
  occupied_pointcloud_pub_.publish(bundle.occupied_pointcloud);
  free_pointcloud_pub_.publish(bundle.free_pointcloud);
}
}  // namespace pointcloud_filter
