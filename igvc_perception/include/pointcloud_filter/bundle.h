#ifndef SRC_POINTCLOUD_BUNDLE_H
#define SRC_POINTCLOUD_BUNDLE_H

#include <pcl/point_cloud.h>
#include <velodyne_pointcloud/point_types.h>

namespace pointcloud_filter
{
struct Bundle
{
  using PointCloud = pcl::PointCloud<velodyne_pointcloud::PointXYZIR>;

  PointCloud::Ptr pointcloud;
  PointCloud::Ptr occupied_pointcloud;
  PointCloud::Ptr free_pointcloud;

  Bundle(const PointCloud::ConstPtr& raw_pointcloud);
};
}  // namespace pointcloud_filter

#endif  // SRC_POINTCLOUD_BUNDLE_H
