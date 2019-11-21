#include "lidar_layer_config.h"

#include <parameter_assertions/assertions.h>

namespace lidar_layer
{
LidarLayerConfig::LidarLayerConfig(const ros::NodeHandle &parent_nh)
  : nh{ parent_nh, "lidar_layer" }, map{ nh }, lidar{ nh }
{
}
}  // namespace lidar_layer
