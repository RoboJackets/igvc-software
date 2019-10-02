#include "lidar_config.h"
#include <mapper/probability_utils.h>
#include <parameter_assertions/assertions.h>

namespace lidar_layer
{
LidarConfig::LidarConfig(const ros::NodeHandle& parent_nh)
{
  ros::NodeHandle nh{ parent_nh, "lidar" };

  assertions::getParam(nh, "occupied_topic", occupied_topic);
  assertions::getParam(nh, "free_topic", free_topic);
  assertions::getParam(nh, "free_topic", free_topic);

  assertions::getParam(nh, "sensor_model/scan_hit", scan_hit);
  // no log odds since conversion is done during markScanHit since its a function of distance

  assertions::getParam(nh, "sensor_model/scan_miss", scan_miss);
  scan_miss = probability_utils::toLogOdds(1.0 - scan_miss);  // It's a miss, so 1 - hit

  assertions::getParam(nh, "sensor_model/free_miss", free_miss);
  free_miss = probability_utils::toLogOdds(1.0 - free_miss);  // It's a miss, so 1 - hit

  assertions::getParam(nh, "sensor_model/hit_exponential_coeff", hit_exponential_coeff);
}

}  // namespace lidar_layer
