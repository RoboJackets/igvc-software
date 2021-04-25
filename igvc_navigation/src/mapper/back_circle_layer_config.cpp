#include "back_circle_layer_config.h"
#include <parameter_assertions/assertions.h>

namespace back_circle_layer
{
BackCircleLayerConfig::BackCircleLayerConfig(const ros::NodeHandle &parent_nh)
{
  ros::NodeHandle nh{parent_nh, "back_circle_layer"};
  assertions::getParam(nh, "topic", topic);
}
} // namespace back_circle_layer