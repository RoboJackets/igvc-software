#ifndef SRC_FAST_SEGMENT_FILTER_CONFIG_H
#define SRC_FAST_SEGMENT_FILTER_CONFIG_H

#include <ros/ros.h>

namespace pointcloud_filter
{
struct FastSegmentFilterConfig
{
  int num_segments;
  double error_t;
  double slope_t;
  double dist_t;
  std::string ground_topic;
  std::string nonground_topic;
  bool debug_viz;

  FastSegmentFilterConfig() = default;
  FastSegmentFilterConfig(const ros::NodeHandle& nh);
};
}  // namespace pointcloud_filter

#endif  // SRC_FAST_SEGMENT_FILTER_CONFIG_H