#ifndef SRC_RADIUS_FILTER_CONFIG_H
#define SRC_RADIUS_FILTER_CONFIG_H

#include <ros/ros.h>

namespace pointcloud_filter
{
struct RadiusFilterConfig
{
  double radius_squared = 0.0;

  RadiusFilterConfig() = default;
  RadiusFilterConfig(const ros::NodeHandle& nh);
};
}  // namespace pointcloud_filter

#endif  // SRC_RADIUS_FILTER_CONFIG_H
