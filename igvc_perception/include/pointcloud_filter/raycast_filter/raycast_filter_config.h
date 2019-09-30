#ifndef SRC_RAYCAST_FILTER_CONFIG_H
#define SRC_RAYCAST_FILTER_CONFIG_H

#include <ros/ros.h>

namespace pointcloud_filter
{
struct RaycastFilterConfig
{
  double end_distance = 0.0;
  double angular_resolution = 0.0;
  double start_angle = 0.0;
  double end_angle = 0.0;

  RaycastFilterConfig() = default;
  RaycastFilterConfig(const ros::NodeHandle& nh);
};
}  // namespace pointcloud_filter

#endif  // SRC_RAYCAST_FILTER_CONFIG_H
