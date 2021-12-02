#ifndef SRC_RADIUS_FILTER_H
#define SRC_RADIUS_FILTER_H

#include <pcl/point_cloud.h>
#include "pointcloud_filter/point_types.h"

#include <pointcloud_filter/filter.h>
#include <pointcloud_filter/radius_filter/radius_filter_config.h>

namespace pointcloud_filter
{
class RadiusFilter : Filter
{
public:
  using PointCloud = pcl::PointCloud<velodyne_pcl::PointXYZIRT>;

  explicit RadiusFilter(const ros::NodeHandle& nh);

  void filter(Bundle& bundle) override;

private:
  RadiusFilterConfig config_{};
};
}  // namespace pointcloud_filter

#endif  // SRC_RADIUS_FILTER_H
