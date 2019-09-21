#include <parameter_assertions/assertions.h>
#include <pointcloud_filter/back_filter/back_filter_config.h>

namespace pointcloud_filter
{
BackFilterConfig::BackFilterConfig(const ros::NodeHandle &nh)
{
  ros::NodeHandle child_nh{ nh, "back_filter" };

  assertions::getParam(child_nh, "start_angle", start_angle);
  assertions::getParam(child_nh, "end_angle", end_angle);
}
}  // namespace pointcloud_filter
