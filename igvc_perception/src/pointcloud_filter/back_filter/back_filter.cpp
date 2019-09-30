#include <pointcloud_filter/back_filter/back_filter.h>

namespace pointcloud_filter
{
BackFilter::BackFilter(const ros::NodeHandle& nh) : config_{ nh }
{
}

void BackFilter::filter(Bundle& bundle)
{
  const auto& start_angle = config_.start_angle;
  const auto& end_angle = config_.end_angle;

  PointCloud filtered_pc;
  filtered_pc.points.reserve(bundle.pointcloud->points.size());

  for (const auto& point : *bundle.pointcloud)
  {
    double angle = atan2(point.y, point.x);
    bool within_angle = end_angle >= angle && angle >= start_angle;
    if (within_angle)
    {
      filtered_pc.points.emplace_back(point);
    }
  }
  bundle.pointcloud->points = std::move(filtered_pc.points);
}
}  // namespace pointcloud_filter
