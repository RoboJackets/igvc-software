#ifndef SRC_BACK_FILTER_H
#define SRC_BACK_FILTER_H

#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pointcloud_filter/back_filter/back_filter_config.h>
#include <velodyne_pointcloud/point_types.h>

namespace pointcloud_filter
{
class BackFilter
{
public:
  using PointCloud = pcl::PointCloud<velodyne_pointcloud::PointXYZIR>;
  explicit BackFilter(const BackFilterConfig& config);

  void filter(PointCloud& pointcloud) const;

private:
  BackFilterConfig config_;
};
}  // namespace pointcloud_filter

#endif  // SRC_BACK_FILTER_H
