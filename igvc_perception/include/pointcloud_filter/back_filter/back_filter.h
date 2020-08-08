#ifndef SRC_BACK_FILTER_H
#define SRC_BACK_FILTER_H

#include <pcl/point_cloud.h>
#include <velodyne_pcl/point_types.h>

#include <pointcloud_filter/back_filter/back_filter_config.h>
#include <pointcloud_filter/filter.h>

namespace pointcloud_filter
{
class BackFilter : Filter
{
public:
  using PointCloud = pcl::PointCloud<velodyne_pcl::PointXYZIRT>;

  explicit BackFilter(const ros::NodeHandle& nh);

  void filter(Bundle& bundle) override;

private:
  BackFilterConfig config_{};
};
}  // namespace pointcloud_filter

#endif  // SRC_BACK_FILTER_H
