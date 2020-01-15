#ifndef SRC_POINTCLOUD_FILTER_H
#define SRC_POINTCLOUD_FILTER_H

#include <pointcloud_filter/back_filter/back_filter.h>
#include <pointcloud_filter/fast_segment_filter/fast_segment_filter.h>
#include <pointcloud_filter/ground_filter/ground_filter.h>
#include <pointcloud_filter/pointcloud_filter_config.h>
#include <pointcloud_filter/radius_filter/radius_filter.h>
#include <pointcloud_filter/raycast_filter/raycast_filter.h>
#include <pointcloud_filter/tf_transform_filter/tf_transform_filter.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace pointcloud_filter
{
class PointcloudFilter
{
public:
  using PointCloud = pcl::PointCloud<velodyne_pointcloud::PointXYZIR>;

  PointcloudFilter(const ros::NodeHandle& nh = {}, const ros::NodeHandle& private_nh = { "~" });

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  PointcloudFilterConfig config_;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;

  BackFilter back_filter_;
  RadiusFilter radius_filter_;
  TFTransformFilter tf_transform_filter_;
  GroundFilter ground_filter_;
  RaycastFilter raycast_filter_;
  FastSegmentFilter fast_segment_filter_;

  ros::Subscriber raw_pointcloud_sub_;

  ros::Publisher transformed_pointcloud_pub_;
  ros::Publisher occupied_pointcloud_pub_;
  ros::Publisher free_pointcloud_pub_;

  void setupPubSub();
  void pointcloudCallback(const PointCloud::ConstPtr& raw_pointcloud);
};
}  // namespace pointcloud_filter

#endif  // SRC_POINTCLOUD_FILTER_H
