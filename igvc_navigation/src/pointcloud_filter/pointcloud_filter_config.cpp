#include <pointcloud_filter/pointcloud_filter_config.h>

namespace pointcloud_filter
{
PointcloudFilterConfig::PointcloudFilterConfig(const ros::NodeHandle &nh)
  : back_filter_config_{ ros::NodeHandle{ nh, "pointcloud_filter" } }
{
}
}  // namespace pointcloud_filter
