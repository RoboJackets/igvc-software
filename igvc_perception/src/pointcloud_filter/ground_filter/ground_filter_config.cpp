#include <parameter_assertions/assertions.h>
#include <pointcloud_filter/ground_filter/ground_filter_config.h>

namespace pointcloud_filter
{
GroundFilterConfig::GroundFilterConfig(const ros::NodeHandle &nh)
{
  ros::NodeHandle child_nh{ nh, "ground_filter" };

  assertions::getParam(child_nh, "height_min", height_min);
  assertions::getParam(child_nh, "height_max", height_max);
}
}  // namespace pointcloud_filter
