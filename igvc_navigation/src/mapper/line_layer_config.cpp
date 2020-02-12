#include "line_layer_config.h"

namespace line_layer
{
LineLayerConfig::LineLayerConfig(const ros::NodeHandle &parent_nh)
  : nh{ parent_nh, "line_layer" }
  , map{ nh }
  , cameras{ { nh, "/cam/center" }, { nh, "/cam/left" }, { nh, "/cam/right" } }
  , projection{ nh, map.resolution }
  , barrelConfig{nh}
{
}
}  // namespace line_layer
