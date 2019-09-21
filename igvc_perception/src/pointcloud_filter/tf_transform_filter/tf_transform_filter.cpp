#include <pcl_conversions/pcl_conversions.h>
#include <pointcloud_filter/tf_transform_filter/tf_transform_filter.h>
#include <tf2_eigen/tf2_eigen.h>

namespace pointcloud_filter
{
TFTransformFilter::TFTransformFilter(const ros::NodeHandle &nh)
  : config_{ nh }, buffer_{}, transform_listener_{ buffer_ }
{
}

void TFTransformFilter::filter(pointcloud_filter::Bundle &bundle)
{
  const auto &source_frame = bundle.pointcloud->header.frame_id;
  const auto &target_frame = config_.target_frame;
  ros::Time message_time = pcl_conversions::fromPCL(bundle.pointcloud->header.stamp);
  ros::Duration timeout{ config_.timeout };

  geometry_msgs::TransformStamped transform_msg =
      buffer_.lookupTransform(target_frame, source_frame, message_time, timeout);
  Eigen::Isometry3f transform = tf2::transformToEigen(transform_msg).cast<float>();

  for (auto &point : *bundle.pointcloud)
  {
    Eigen::Vector3f eigen_point{ point.x, point.y, point.z };
    eigen_point = transform * eigen_point;

    point.x = eigen_point(0);
    point.y = eigen_point(1);
    point.z = eigen_point(2);
  }

  bundle.pointcloud->header.frame_id = target_frame;
}
}  // namespace pointcloud_filter
