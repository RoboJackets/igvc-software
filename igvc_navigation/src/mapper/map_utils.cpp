#include <unordered_set>

#include "map_utils.h"

namespace MapUtils
{
inline int discretize(radians angle, double angular_resolution)
{
  double coeff = 1 / angular_resolution;
  return static_cast<int>(std::round(coeff * angle));
}

void filterPointsBehind(const pcl::PointCloud<pcl::PointXYZ>& pc, pcl::PointCloud<pcl::PointXYZ>& filtered_pc,
                        BehindFilterOptions options)
{
  static double start_angle = -M_PI + options.angle / 2;
  static double end_angle = M_PI - options.angle / 2;
  static double squared_distance = options.distance * options.distance;
  // Iterate over pointcloud, insert discretized angles into set
  for (auto i : pc)
  {
    double angle = atan2(i.y, i.x);
    if ((-M_PI <= angle && angle < start_angle) || (end_angle < angle && angle <= M_PI))
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

void blur(cv::Mat& blurred_map, double kernel_size)
{
  cv::Mat original = blurred_map.clone();
  cv::blur(blurred_map, blurred_map, cv::Size(kernel_size, kernel_size));
  cv::max(original, blurred_map, blurred_map);
}

void getEmptyPoints(const pcl::PointCloud<pcl::PointXYZ>& pc, pcl::PointCloud<pcl::PointXYZ>& empty_pc,
                    double angular_resolution, EmptyFilterOptions options)
{
  // Iterate over pointcloud, insert discretized angles into set
  std::unordered_set<int> discretized_angles{};
  for (auto i : pc)
  {
    double angle = atan2(i.y, i.x);
    discretized_angles.emplace(MapUtils::discretize(angle, angular_resolution));
  }

  // For each angle, if it's not in the set (empty), put it into a pointcloud
  static double coeff = 1 / angular_resolution;
  // From Robot's frame. Need to rotate angle to world frame
  for (int i = static_cast<int>(options.start_angle * coeff); i < options.end_angle * coeff; i++)
  {
    if (discretized_angles.find(i) == discretized_angles.end())
    {
      double angle = i * angular_resolution;
      pcl::PointXYZ point{ static_cast<float>(options.miss_cast_distance * cos(angle)),
                           static_cast<float>(options.miss_cast_distance * sin(angle)), 0 };
      empty_pc.points.emplace_back(point);
    }
  }
}

}  // namespace MapUtils
