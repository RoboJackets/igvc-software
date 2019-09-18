#include <pointcloud_filter/back_filter/back_filter.h>

namespace pointcloud_filter
{
BackFilter::BackFilter(const BackFilterConfig& config) : config_{ config }
{
}

void BackFilter::filter(PointCloud& pointcloud) const
{
  const auto& start_angle = config_.start_angle;
  const auto& end_angle = config_.start_angle;
  for (const auto& point : pointcloud)
  {
    double angle = atan2(point.y, point.x);
    if ((-M_PI <= angle && angle < end_angle) || (start_angle < angle && angle <= M_PI))
    {
      if (i.x * i.x + i.y * i.y > squared_distance)
      {
        filtered_pc.points.emplace_back(i);
      }
    }
    else
    {
      filtered_pc.points.emplace_back(i);
    }
  }
}
}  // namespace pointcloud_filter
