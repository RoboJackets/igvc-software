//
// Created by pcapple on 2/2/20.
//

#ifndef SRC_BARREL_CONFIG_H
#define SRC_BARREL_CONFIG_H

#include <ros/ros.h>

namespace line_layer
{
class BarrelConfig
{
public:
  BarrelConfig(const ros::NodeHandle& parent_nh);

  const ros::NodeHandle parent_nh;

  int blur_size;

  int min_h;
  int min_s;
  int min_v;

  int max_h;
  int max_s;
  int max_v;

  int barrel_value;

  bool debug;
};
}  // namespace line_layer
#endif  // SRC_BARREL_CONFIG_H
