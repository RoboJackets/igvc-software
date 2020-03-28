#ifndef SRC_ROLLING_LAYER_CONFIG_H
#define SRC_ROLLING_LAYER_CONFIG_H

#include <ros/ros.h>

namespace rolling_layer
{
class RollingLayerConfig
{
public:
  RollingLayerConfig(const ros::NodeHandle& parent_nh);

  std::string topic;
};
}  // namespace rolling_layer

#endif  // SRC_ROLLING_LAYER_CONFIG_H
