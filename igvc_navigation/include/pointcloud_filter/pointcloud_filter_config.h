#ifndef SRC_POINTCLOUD_FILTER_CONFIG_H
#define SRC_POINTCLOUD_FILTER_CONFIG_H

#include <ros/ros.h>

#include <pointcloud_filter/back_filter/back_filter_config.h>

namespace pointcloud_filter
{
struct PointcloudFilterConfig
{
  PointcloudFilterConfig() = default;
  explicit PointcloudFilterConfig(const ros::NodeHandle& nh);
};
}  // namespace pointcloud_filter

#endif  // SRC_POINTCLOUD_FILTER_CONFIG_H
