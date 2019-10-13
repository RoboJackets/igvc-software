#ifndef SRC_GROUND_FILTER_CONFIG_H
#define SRC_GROUND_FILTER_CONFIG_H

#include <ros/ros.h>

namespace pointcloud_filter
{
struct GroundFilterConfig
{
  double height_min = 0.0;
  double height_max = 0.0;

  GroundFilterConfig() = default;
  GroundFilterConfig(const ros::NodeHandle& nh);
};
}  // namespace pointcloud_filter

#endif  // SRC_GROUND_FILTER_CONFIG_H
