#include "traversability_layer_config.h"
#include <parameter_assertions/assertions.h>

namespace traversability_layer
{
TraversabilityLayerConfig::TraversabilityLayerConfig(const ros::NodeHandle& parent_nh)
  : nh(parent_nh, "traversability_layer"), map(parent_nh)
{
  assertions::getParam(nh, "logodd_increment", logodd_increment);
}
}  // namespace traversability_layer
