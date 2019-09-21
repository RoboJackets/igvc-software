#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <parameter_assertions/assertions.h>

#include <pointcloud_filter/pointcloud_filter.h>

namespace pointcloud_filter
{
PointcloudFilter::PointcloudFilter(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
  : nh_{ nh }
  , private_nh_{ private_nh, "pointcloud_filter" }
  , config_{ private_nh_ }
  , back_filter_{ private_nh_ }
  , radius_filter_{ private_nh_ }
  , tf_transform_filter_{ private_nh_ }
  , ground_filter_{ private_nh_ }
{
  setupPubSub();
}

void PointcloudFilter::setupPubSub()
{
  raw_pointcloud_sub_ = nh_.subscribe(config_.topic_input, 1, &PointcloudFilter::pointcloudCallback, this);

  transformed_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(config_.topic_transformed, 1);
  occupied_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(config_.topic_occupied, 1);
  free_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(config_.topic_free, 1);
}

void PointcloudFilter::pointcloudCallback(const PointCloud::ConstPtr& raw_pointcloud)
{
  Bundle bundle{ raw_pointcloud };

  radius_filter_.filter(bundle);

  back_filter_.filter(bundle);

  tf_transform_filter_.filter(bundle);

  bundle.occupied_pointcloud->header = bundle.pointcloud->header;
  bundle.free_pointcloud->header = bundle.pointcloud->header;

  ground_filter_.filter(bundle);

  transformed_pointcloud_pub_.publish(bundle.pointcloud);
  occupied_pointcloud_pub_.publish(bundle.occupied_pointcloud);
}
}  // namespace pointcloud_filter
