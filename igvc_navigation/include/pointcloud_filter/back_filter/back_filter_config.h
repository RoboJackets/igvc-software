#ifndef SRC_BACK_FILTER_CONFIG_H
#define SRC_BACK_FILTER_CONFIG_H

#include <ros/ros.h>

namespace pointcloud_filter
{
struct BackFilterConfig
{
  double start_angle;
  double end_angle;

  BackFilterConfig() = default;
  BackFilterConfig(const ros::NodeHandle& nh);
};
}  // namespace pointcloud_filter

#endif  // SRC_BACK_FILTER_CONFIG_H
