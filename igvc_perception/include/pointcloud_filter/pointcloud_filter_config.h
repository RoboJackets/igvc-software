#ifndef SRC_POINTCLOUD_FILTER_CONFIG_H
#define SRC_POINTCLOUD_FILTER_CONFIG_H

#include <pointcloud_filter/back_filter/back_filter_config.h>
#include <ros/ros.h>

namespace pointcloud_filter
{
struct PointcloudFilterConfig
{
  PointcloudFilterConfig() = default;
  explicit PointcloudFilterConfig(const ros::NodeHandle& nh);

  std::string topic_input;
  std::string topic_transformed;
  std::string topic_occupied;
  std::string topic_free;
  std::string topic_filtered;

  std::string base_frame;

  double timeout_duration;
};
}  // namespace pointcloud_filter

#endif  // SRC_POINTCLOUD_FILTER_CONFIG_H
