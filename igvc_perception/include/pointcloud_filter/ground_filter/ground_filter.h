#ifndef SRC_GROUND_FILTER_H
#define SRC_GROUND_FILTER_H

#include <pcl/point_cloud.h>
#include <velodyne_pointcloud/point_types.h>

#include <pointcloud_filter/filter.h>
#include <pointcloud_filter/ground_filter/ground_filter_config.h>

namespace pointcloud_filter
{
class GroundFilter : Filter
{
public:
  using PointCloud = pcl::PointCloud<velodyne_pointcloud::PointXYZIR>;

  explicit GroundFilter(const ros::NodeHandle& nh);

  void filter(Bundle& bundle) override;

private:
  GroundFilterConfig config_{};
};
}  // namespace pointcloud_filter

#endif  // SRC_GROUND_FILTER_H
