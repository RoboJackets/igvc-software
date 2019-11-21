#include <parameter_assertions/assertions.h>
#include <pointcloud_filter/fast_segment_filter/fast_segment_filter_config.h>
#include <ros/ros.h>

namespace pointcloud_filter
{
FastSegmentFilterConfig::FastSegmentFilterConfig(const ros::NodeHandle& nh)
{
  ros::NodeHandle child_nh{ nh, "fast_segment_filter" };

  assertions::getParam(child_nh, "ground_topic", ground_topic);
  assertions::getParam(child_nh, "nonground_topic", nonground_topic);
  assertions::getParam(child_nh, "num_segments", num_segments);
  assertions::getParam(child_nh, "error_t", error_t);
  assertions::getParam(child_nh, "slope_t", slope_t);
  assertions::getParam(child_nh, "intercept_z_t", intercept_z_t);
  assertions::getParam(child_nh, "dist_t", dist_t);
  assertions::getParam(child_nh, "debug_viz", debug_viz);
}
}  // namespace pointcloud_filter