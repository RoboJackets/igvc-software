#ifndef SRC_TF_TRANSFORM_FILTER_CONFIG_H
#define SRC_TF_TRANSFORM_FILTER_CONFIG_H

#include <ros/ros.h>

namespace pointcloud_filter
{
struct TFTransformFilterConfig
{
  std::string target_frame = "";
  double timeout = 0.0;

  TFTransformFilterConfig() = default;
  TFTransformFilterConfig(const ros::NodeHandle& nh);
};
}  // namespace pointcloud_filter

#endif  // SRC_TF_TRANSFORM_FILTER_CONFIG_H
