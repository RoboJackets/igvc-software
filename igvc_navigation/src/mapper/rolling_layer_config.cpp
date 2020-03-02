#include "rolling_layer_config.h"
#include <parameter_assertions/assertions.h>

namespace rolling_layer
{
RollingLayerConfig::RollingLayerConfig(const ros::NodeHandle &parent_nh)
{
  ros::NodeHandle nh(parent_nh, "rolling_layer");

  assertions::getParam(nh, "topic", topic);
}
}  // namespace rolling_layer
