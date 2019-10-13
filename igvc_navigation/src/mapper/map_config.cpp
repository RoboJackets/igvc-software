#include "map_config.h"
#include <mapper/probability_utils.h>
#include <parameter_assertions/assertions.h>

namespace map
{
MapConfig::MapConfig(const ros::NodeHandle &parent_nh)
{
  ros::NodeHandle nh{ parent_nh, "map" };

  assertions::getParam(nh, "resolution", resolution);
  assertions::getParam(nh, "length_x", length_x);
  assertions::getParam(nh, "length_y", length_y);

  assertions::getParam(nh, "frame_id", frame_id);
  assertions::getParam(nh, "occupied_threshold", occupied_threshold);

  assertions::getParam(nh, "max_occupancy", max_occupancy);
  max_occupancy = probability_utils::toLogOdds(max_occupancy);
  assertions::getParam(nh, "min_occupancy", min_occupancy);
  min_occupancy = probability_utils::toLogOdds(min_occupancy);

  assertions::getParam(nh, "debug/map_topic", debug.map_topic);
}
}  // namespace map
