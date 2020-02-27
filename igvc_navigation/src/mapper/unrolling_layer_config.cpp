#include <parameter_assertions/assertions.h>

#include "unrolling_layer_config.h"

namespace unrolling_layer
{
UnrollingLayerConfig::UnrollingLayerConfig(const ros::NodeHandle &parent_nh)
{
  ros::NodeHandle nh{ parent_nh, "unrolling_layer" };

  assertions::getParam(nh, "topic", topic);
}
}  // namespace unrolling_layer
