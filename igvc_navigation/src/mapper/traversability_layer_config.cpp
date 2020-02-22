#include "traversability_layer_config.h"

namespace traversability_layer
{
TraversabilityLayerConfig::TraversabilityLayerConfig(const ros::NodeHandle& parent_nh)
  : nh(parent_nh, "traversability_layer"), map(parent_nh)
{
}
}  // namespace traversability_layer
