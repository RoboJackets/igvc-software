#include "traversability_layer_config.h"
#include <parameter_assertions/assertions.h>
#include <mapper/probability_utils.h>

namespace traversability_layer
{
TraversabilityLayerConfig::TraversabilityLayerConfig(const ros::NodeHandle &parent_nh)
  : nh(parent_nh, "traversability_layer"), map(nh)
{
  double untraversable_probability;
  assertions::getParam(nh, "untraversable_probability", untraversable_probability);
  assertions::getParam(nh, "slope_threshold", slope_threshold);

  logodd_increment = probability_utils::toLogOdds(untraversable_probability);
}
}  // namespace traversability_layer
