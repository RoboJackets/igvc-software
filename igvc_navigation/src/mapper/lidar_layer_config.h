#ifndef SRC_LIDAR_LAYER_CONFIG_H
#define SRC_LIDAR_LAYER_CONFIG_H

#include <ros/ros.h>
#include "lidar_config.h"
#include "map_config.h"

namespace lidar_layer
{
class LidarLayerConfig
{
public:
  explicit LidarLayerConfig(const ros::NodeHandle& parent_nh);

  ros::NodeHandle nh;

  MapConfig map;
  LidarConfig lidar;
};
}  // namespace lidar_layer

#endif  // SRC_LIDAR_LAYER_CONFIG_H
