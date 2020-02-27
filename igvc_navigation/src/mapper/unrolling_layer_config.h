#ifndef SRC_UNROLLING_LAYER_CONFIG_H
#define SRC_UNROLLING_LAYER_CONFIG_H

#include <ros/ros.h>

namespace unrolling_layer
{
struct UnrollingLayerConfig
{
  UnrollingLayerConfig(const ros::NodeHandle& parent_nh);

  std::string topic;
};
}  // namespace unrolling_layer

#endif  // SRC_UNROLLING_LAYER_CONFIG_H
