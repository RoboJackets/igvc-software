#include "projection_config.h"
#include <parameter_assertions/assertions.h>

namespace line_layer
{
ProjectionConfig::ProjectionConfig(const ros::NodeHandle &parent_nh, double resolution)
{
  ros::NodeHandle nh{ parent_nh, "projection" };

  assertions::getParam(nh, "length_x", length_x);
  assertions::getParam(nh, "length_y", length_y);

  assertions::getParam(nh, "line_closing_kernel_size", line_closing_kernel_size);
  assertions::getParam(nh, "freespace_closing_kernel_size", freespace_closing_kernel_size);

  size_x = static_cast<int>(std::round(length_x / resolution));
  size_y = static_cast<int>(std::round(length_y / resolution));
}
}  // namespace line_layer
