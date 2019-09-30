#include <parameter_assertions/assertions.h>
#include <pointcloud_filter/radius_filter/radius_filter_config.h>

namespace pointcloud_filter
{
RadiusFilterConfig::RadiusFilterConfig(const ros::NodeHandle &nh)
{
  ros::NodeHandle child_nh{ nh, "radius_filter" };

  assertions::getParam(child_nh, "radius_squared", radius_squared);
}
}  // namespace pointcloud_filter
