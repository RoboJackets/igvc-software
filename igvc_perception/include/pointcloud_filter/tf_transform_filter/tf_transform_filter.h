
#ifndef SRC_TF_TRANSFORM_FILTER_H
#define SRC_TF_TRANSFORM_FILTER_H

#include <pcl/point_cloud.h>
#include <tf2_ros/buffer.h>
#include "pointcloud_filter/point_types.h"

#include <pointcloud_filter/filter.h>
#include <tf2_ros/transform_listener.h>

namespace pointcloud_filter
{
class TFTransformFilter
{
public:
  using PointCloud = pcl::PointCloud<velodyne_pcl::PointXYZIRT>;

  explicit TFTransformFilter(tf2_ros::Buffer* buffer_);

  void transform(const PointCloud& from, PointCloud& to, const std::string& target_frame, const ros::Duration& timeout);

private:
  tf2_ros::Buffer* buffer_;
};
}  // namespace pointcloud_filter

#endif  // SRC_TF_TRANSFORM_FILTER_H
