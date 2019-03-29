#include "mapper.h"
#include <pcl_ros/transforms.h>
#include "map_utils.h"

void Mapper::insertLidarScan(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc, const tf::Transform& odom_to_lidar)
{
  pcl::PointCloud<pcl::PointXYZ> empty_pc{};
  pcl::PointCloud<pcl::PointXYZ> filtered_pc{};
  MapUtils::getEmptyPoints(*pc, empty_pc, angular_resolution_, empty_filter_options_);
  MapUtils::filterPointsBehind(*pc, filtered_pc, behind_filter_options_);

  // Apply transformation from lidar to base_link aka robot pose
  pcl_ros::transformPointCloud(filtered_pc, filtered_pc, odom_to_lidar);
  pcl_ros::transformPointCloud(empty_pc, empty_pc, odom_to_lidar);

  octomapper_->insert_scan(odom_to_lidar.getOrigin(), pc_map_pair_, filtered_pc, empty_pc);

  // Get updated map from octomapper
  octomapper_->get_updated_map(pc_map_pair_);
}

void Mapper::insertCameraProjection(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc,
                                    const tf::Transform& odom_to_base)
{
  pcl::PointCloud<pcl::PointXYZ> transformed{};

  pcl_ros::transformPointCloud(*pc, transformed, odom_to_base);
  transformed.header.stamp = pc->header.stamp;
  transformed.header.frame_id = "odom";
  camera_projection_pub.publish(transformed);

  octomapper_->insertCameraProjection(camera_map_pair_, transformed, true);
  //
  //  // Get updated map from octomapper
  octomapper_->get_updated_map(camera_map_pair_);
}

void Mapper::insertSegmentedImage(const cv::Mat& image, const tf::Transform& odom_to_camera)
{
  // Insert into octree
  octomapper_->insert_camera_free(camera_map_pair_, image, camera_model_, odom_to_camera);

  // Get updated map from octomapper
  octomapper_->get_updated_map(camera_map_pair_);
}

std::shared_ptr<cv::Mat> Mapper::getMap()
{
  return std::shared_ptr<cv::Mat>();
}

void Mapper::setProjectionModel(const image_geometry::PinholeCameraModel&& camera_model)
{
  camera_model_ = camera_model;
  camera_model_initialized_ = true;
}

Mapper::Mapper(const ros::NodeHandle& pNh)
{
  igvc::getParam(pNh, "sensor_model/max_range", radius_);
  igvc::getParam(pNh, "sensor_model/angular_resolution", angular_resolution_);

  igvc::getParam(pNh, "sensor_model/lidar_miss_cast_distance", empty_filter_options_.miss_cast_distance);
  igvc::getParam(pNh, "sensor_model/lidar_angle_start", empty_filter_options_.start_angle);
  igvc::getParam(pNh, "sensor_model/lidar_angle_end", empty_filter_options_.end_angle);

  igvc::getParam(pNh, "filter/filter_angle", behind_filter_options_.angle);
  igvc::getParam(pNh, "filter/distance", behind_filter_options_.distance);

  octomapper_ = std::make_unique<Octomapper>(pNh);
  octomapper_->create_octree(pc_map_pair_);
  octomapper_->create_octree(camera_map_pair_);
}
