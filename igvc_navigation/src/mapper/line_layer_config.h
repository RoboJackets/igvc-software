#ifndef SRC_LINE_LAYER_CONFIG_H
#define SRC_LINE_LAYER_CONFIG_H

#include <ros/ros.h>
#include "camera_config.h"
#include "map_config.h"
#include "projection_config.h"
#include "barrel_config.h"

namespace line_layer
{
class LineLayerConfig
{
public:
  explicit LineLayerConfig(const ros::NodeHandle& parent_nh);

  ros::NodeHandle nh;

  map::MapConfig map;
  std::vector<CameraConfig> cameras;
  ProjectionConfig projection;
  BarrelConfig barrelConfig;

};

}  // namespace line_layer

#endif  // SRC_LINE_LAYER_CONFIG_H
