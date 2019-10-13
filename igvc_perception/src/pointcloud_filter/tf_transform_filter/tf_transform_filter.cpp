#include <pcl_conversions/pcl_conversions.h>
#include <pointcloud_filter/tf_transform_filter/tf_transform_filter.h>
#include <tf2_eigen/tf2_eigen.h>

namespace pointcloud_filter
{
TFTransformFilter::TFTransformFilter(tf2_ros::Buffer* buffer) : buffer_{ buffer }
{
}

void TFTransformFilter::transform(const PointCloud& from, PointCloud& to, const std::string& target_frame,
                                  const ros::Duration& timeout)
{
  if (&from != &to)
  {
    to.clear();
    to.reserve(from.size());
  }

  const auto& source_frame = from.header.frame_id;
  ros::Time message_time = pcl_conversions::fromPCL(from.header.stamp);

  geometry_msgs::TransformStamped transform_msg;
  if (buffer_->canTransform(target_frame, source_frame, message_time, timeout))
  {
    transform_msg = buffer_->lookupTransform(target_frame, source_frame, message_time, timeout);
  }
  else
  {
    transform_msg = buffer_->lookupTransform(target_frame, source_frame, ros::Time(0), timeout);
  }
  Eigen::Isometry3f transform = tf2::transformToEigen(transform_msg).cast<float>();

  for (size_t i = 0; i < from.size(); i++)
  {
    auto& point = from.points[i];
    Eigen::Vector3f eigen_point{ point.x, point.y, point.z };
    eigen_point = transform * eigen_point;

    if (&from == &to)
    {
      auto& to_point = to.points[i];
      to_point.x = eigen_point(0);
      to_point.y = eigen_point(1);
      to_point.z = eigen_point(2);
    }
    else
    {
      velodyne_pointcloud::PointXYZIR new_point{ point };
      new_point.x = eigen_point(0);
      new_point.y = eigen_point(1);
      new_point.z = eigen_point(2);
      to.points.emplace_back(new_point);
    }
  }

  to.header.frame_id = target_frame;
}
}  // namespace pointcloud_filter
