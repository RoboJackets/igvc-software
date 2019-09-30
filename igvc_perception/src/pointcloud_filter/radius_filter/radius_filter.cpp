#include <pointcloud_filter/radius_filter/radius_filter.h>

namespace pointcloud_filter
{
RadiusFilter::RadiusFilter(const ros::NodeHandle& nh) : config_{ nh }
{
}

void RadiusFilter::filter(pointcloud_filter::Bundle& bundle)
{
  const auto& radius_squared = config_.radius_squared;

  PointCloud filtered_pc;
  filtered_pc.points.reserve(bundle.pointcloud->points.size());

  for (const auto& point : *bundle.pointcloud)
  {
    double point_radius_squared = point.x * point.x + point.y * point.y;
    bool within_radius = point_radius_squared <= radius_squared;
    if (within_radius)
    {
      filtered_pc.points.emplace_back(point);
    }
  }
  bundle.pointcloud->points = std::move(filtered_pc.points);
}
}  // namespace pointcloud_filter
