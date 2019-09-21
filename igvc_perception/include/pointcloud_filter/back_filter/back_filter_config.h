#ifndef SRC_BACK_FILTER_CONFIG_H
#define SRC_BACK_FILTER_CONFIG_H

#include <ros/ros.h>

namespace pointcloud_filter
{
struct BackFilterConfig
{
  double start_angle = 0.0;
  double end_angle = 0.0;

  BackFilterConfig() = default;
  BackFilterConfig(const ros::NodeHandle& nh);
};
}  // namespace pointcloud_filter

#endif  // SRC_BACK_FILTER_CONFIG_H
