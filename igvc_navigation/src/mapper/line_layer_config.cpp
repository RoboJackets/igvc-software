#include "line_layer_config.h"

namespace line_layer
{
LineLayerConfig::LineLayerConfig(const ros::NodeHandle &parent_nh)
  : nh{ parent_nh, "line_layer" }, map{ nh }, center{ nh, "/cam/center" }
{
}
}  // namespace line_layer
