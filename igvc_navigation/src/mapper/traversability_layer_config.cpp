#include "traversability_layer_config.h"
#include <parameter_assertions/assertions.h>

namespace traversability_layer
{
TraversabilityLayerConfig::TraversabilityLayerConfig(const ros::NodeHandle &parent_nh)
  : nh(parent_nh, "traversability_layer"), map(nh)
{
  assertions::getParam(nh, "logodd_increment", logodd_increment);
  assertions::getParam(nh, "slope_threshold", slope_threshold);
}
}  // namespace traversability_layer
