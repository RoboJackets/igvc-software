#ifndef SRC_BACK_CIRCLE_LAYER_CONFIG_H
#define SRC_BACK_CIRCLE_LAYER_CONFIG_H

#include <ros/ros.h>

namespace back_circle_layer
{
class BackCircleLayerConfig
{
public:
  BackCircleLayerConfig(const ros::NodeHandle& parent_nh);
  std::string topic;
};
} // namespace back_circle_layer

#endif // SRC_BACK_CIRCLE_LAYER_CONFIG_H