#ifndef SRC_TRAVERSABILITY_CONFIG_H
#define SRC_TRAVERSABILITY_CONFIG_H

#include <ros/ros.h>
#include "map_config.h"

namespace traversability_layer
{
class TraversabilityLayerConfig
{
public:
  explicit TraversabilityLayerConfig(const ros::NodeHandle& parent_nh);

  ros::NodeHandle nh;

  map::MapConfig map;

  double logodd_increment;
  double slope_threshold;
};
}  // namespace traversability_layer

#endif  // SRC_TRAVERSABILITY_CONFIG_H
