#ifndef SRC_LIDAR_CONFIG_H
#define SRC_LIDAR_CONFIG_H

#include <ros/ros.h>

namespace lidar_layer
{
class LidarConfig
{
public:
  LidarConfig(const ros::NodeHandle& parent_nh);

  std::string occupied_topic;
  std::string free_topic;

  double scan_miss;
  double scan_hit;
  double free_miss;
  double hit_exponential_coeff;
};
}  // namespace lidar_layer

#endif  // SRC_LIDAR_CONFIG_H
