#include <pointcloud_filter/raycast_filter/raycast_filter.h>
#include <unordered_set>

namespace pointcloud_filter
{
RaycastFilter::RaycastFilter(const ros::NodeHandle& nh) : config_{ nh }
{
}

void RaycastFilter::filter(pointcloud_filter::Bundle& bundle)
{
  // Precondition: bundle.occupied_pointcloud has lidar frame

  const auto& min_range = config_.min_range;
  const auto& end_distance = config_.end_distance;
  const auto& start_angle = config_.start_angle;
  const auto& end_angle = config_.end_angle;
  const auto& angular_resolution = config_.angular_resolution;

  int discretized_start = discretize(start_angle);
  int discretized_end = discretize(end_angle);

  bundle.free_pointcloud->points.reserve(discretized_end - discretized_start + 1);

  std::unordered_set<int> discretized_angles{};
  for (auto point : *bundle.occupied_pointcloud)
  {
    double angle = atan2(point.y, point.x);
    discretized_angles.emplace(discretize(angle));
  }

  // For each angle, if it's not in the set (empty), put it into a pointcloud.
  // From Robot's frame. Need to rotate angle to world frame
  for (int i = discretized_start; i < discretized_end; i++)
  {
    if (discretized_angles.find(i) == discretized_angles.end())
    {
      double angle = i * angular_resolution;
      {
        auto x = static_cast<float>(min_range * cos(angle));
        auto y = static_cast<float>(min_range * sin(angle));
        velodyne_pcl::PointXYZIRT min_range_point{ { { x, y, 0.0, 0.0 } }, 0.0, 0 };
        bundle.free_pointcloud->points.emplace_back(min_range_point);
      }
      {
        auto x = static_cast<float>(end_distance * cos(angle));
        auto y = static_cast<float>(end_distance * sin(angle));
        velodyne_pcl::PointXYZIRT raycasted_point{ { { x, y, 0.0, 0.0 } }, 0.0, 0 };
        bundle.free_pointcloud->points.emplace_back(raycasted_point);
      }
    }
  }

  bundle.free_pointcloud->header = bundle.occupied_pointcloud->header;
}

int RaycastFilter::discretize(double angle) const
{
  double discretized = std::round(angle / config_.angular_resolution);
  return static_cast<int>(discretized);
}

}  // namespace pointcloud_filter
