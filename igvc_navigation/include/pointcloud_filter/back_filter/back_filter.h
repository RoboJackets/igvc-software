#ifndef SRC_BACK_FILTER_H
#define SRC_BACK_FILTER_H

#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pointcloud_filter/back_filter/back_filter_config.h>

namespace pointcloud_filter
{
class BackFilter
{
public:
  using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
  explicit BackFilter(const BackFilterConfig& config);

  void filter(PointCloud& pointcloud) const;

private:
  BackFilterConfig config_;
};
}  // namespace pointcloud_filter

#endif  // SRC_BACK_FILTER_H
