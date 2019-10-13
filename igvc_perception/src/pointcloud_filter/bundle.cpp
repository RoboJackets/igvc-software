#include <pointcloud_filter/bundle.h>

namespace pointcloud_filter
{
Bundle::Bundle(const PointCloud::ConstPtr& raw_pointcloud)
  : pointcloud{ new PointCloud }, occupied_pointcloud{ new PointCloud }, free_pointcloud{ new PointCloud }
{
  pointcloud->points = raw_pointcloud->points;
  pointcloud->header = raw_pointcloud->header;
}
}  // namespace pointcloud_filter
