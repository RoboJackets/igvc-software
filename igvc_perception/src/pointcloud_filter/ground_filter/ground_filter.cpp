#include <pointcloud_filter/ground_filter/ground_filter.h>

namespace pointcloud_filter
{
GroundFilter::GroundFilter(const ros::NodeHandle& nh) : config_{ nh }
{
}

void GroundFilter::filter(pointcloud_filter::Bundle& bundle)
{
  const auto& height_min = config_.height_min;
  const auto& height_max = config_.height_max;

  PointCloud filtered_pc;
  filtered_pc.points.reserve(bundle.pointcloud->points.size());

  for (const auto& point : *bundle.pointcloud)
  {
    bool within_thresholds = height_min <= point.z && point.z <= height_max;
    if (within_thresholds)
    {
      filtered_pc.points.emplace_back(point);
    }
  }
  bundle.occupied_pointcloud->points = std::move(filtered_pc.points);
}
}  // namespace pointcloud_filter
