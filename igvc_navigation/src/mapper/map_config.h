#ifndef SRC_MAP_CONFIG_H
#define SRC_MAP_CONFIG_H

#include <ros/ros.h>

namespace map
{
class MapConfig
{
public:
  MapConfig(const ros::NodeHandle& parent_nh);

  double resolution;
  double length_x;
  double length_y;

  double max_occupancy;
  double min_occupancy;

  std::string frame_id;

  double occupied_threshold;

  struct
  {
    std::string map_topic;
  } debug;
};
}  // namespace map

#endif  // SRC_MAP_CONFIG_H
