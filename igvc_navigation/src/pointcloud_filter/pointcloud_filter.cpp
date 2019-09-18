#include <pointcloud_filter/pointcloud_filter.h>

namespace pointcloud_filter
{
PointcloudFilter::PointcloudFilter(const ros::NodeHandle &nh) : nh_{ nh }, config_{ nh_ }
{
  setupPubSub();
}
}  // namespace pointcloud_filter
