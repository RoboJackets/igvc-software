#ifndef SRC_RAYCAST_FILTER_H
#define SRC_RAYCAST_FILTER_H

#include <pcl/point_cloud.h>
#include <velodyne_pointcloud/point_types.h>

#include <pointcloud_filter/filter.h>
#include <pointcloud_filter/raycast_filter/raycast_filter_config.h>

namespace pointcloud_filter
{
class RaycastFilter : Filter
{
public:
  using PointCloud = pcl::PointCloud<velodyne_pointcloud::PointXYZIR>;

  explicit RaycastFilter(const ros::NodeHandle& nh);

  void filter(Bundle& bundle) override;

private:
  RaycastFilterConfig config_;

  int discretize(double angle) const;
};
}  // namespace pointcloud_filter

#endif  // SRC_RAYCAST_FILTER_H
