
#ifndef SRC_TF_TRANSFORM_FILTER_H
#define SRC_TF_TRANSFORM_FILTER_H

#include <pcl/point_cloud.h>
#include <tf2_ros/buffer.h>
#include <velodyne_pointcloud/point_types.h>

#include <pointcloud_filter/filter.h>
#include <pointcloud_filter/tf_transform_filter/tf_transform_filter_config.h>
#include <tf2_ros/transform_listener.h>

namespace pointcloud_filter
{
class TFTransformFilter : Filter
{
public:
  using PointCloud = pcl::PointCloud<velodyne_pointcloud::PointXYZIR>;

  explicit TFTransformFilter(const ros::NodeHandle& nh);

  void filter(Bundle& bundle) override;

private:
  TFTransformFilterConfig config_{};

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener transform_listener_;
};
}  // namespace pointcloud_filter

#endif  // SRC_TF_TRANSFORM_FILTER_H
