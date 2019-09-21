#include <parameter_assertions/assertions.h>
#include <pointcloud_filter/raycast_filter/raycast_filter_config.h>

namespace pointcloud_filter
{
RaycastFilterConfig::RaycastFilterConfig(const ros::NodeHandle &nh)
{
  ros::NodeHandle child_nh{ nh, "raycast_filter" };

  assertions::getParam(child_nh, "end_distance", end_distance);
  assertions::getParam(child_nh, "angular_resolution", angular_resolution);
  assertions::getParam(child_nh, "start_angle", start_angle);
  assertions::getParam(child_nh, "end_angle", end_angle);
}
}  // namespace pointcloud_filter
