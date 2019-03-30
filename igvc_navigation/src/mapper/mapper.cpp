#include "mapper.h"
#include <pcl_ros/transforms.h>
#include "map_utils.h"

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

void Mapper::insertLidarScan(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc, const tf::Transform& odom_to_lidar)
{
  pcl::PointCloud<pcl::PointXYZ> empty_pc{};
  pcl::PointCloud<pcl::PointXYZ> filtered_pc{};

  MapUtils::getEmptyPoints(*pc, empty_pc, angular_resolution_, empty_filter_options_);
  MapUtils::filterPointsBehind(*pc, filtered_pc, behind_filter_options_);

  pcl_ros::transformPointCloud(filtered_pc, filtered_pc, odom_to_lidar);
  pcl_ros::transformPointCloud(empty_pc, empty_pc, odom_to_lidar);

  if (use_ground_filter_)
  {
    pcl::PointCloud<pcl::PointXYZ> ground;
    pcl::PointCloud<pcl::PointXYZ> nonground;
    std::optional<GroundPlane> opt_ground_plane =
        MapUtils::filterGroundPlane(filtered_pc, ground, nonground, ground_filter_options_);
    if (opt_ground_plane)
    {
      ground_plane_ = *opt_ground_plane;
    }
    octomapper_->insertScan(odom_to_lidar.getOrigin(), pc_map_pair_, nonground, lidar_probability_model_);
    octomapper_->insertRays(odom_to_lidar.getOrigin(), pc_map_pair_, ground, false, lidar_probability_model_);
  }
  else
  {
    octomapper_->insertScan(odom_to_lidar.getOrigin(), pc_map_pair_, filtered_pc, lidar_probability_model_);
  }
  octomapper_->insertRays(odom_to_lidar.getOrigin(), pc_map_pair_, empty_pc, false, lidar_probability_model_);

  octomapper_->get_updated_map(camera_map_pair_);
}

void Mapper::insertCameraProjection(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc,
                                    const tf::Transform& odom_to_base)
{
  pcl::PointCloud<pcl::PointXYZ> transformed{};

  pcl_ros::transformPointCloud(*pc, transformed, odom_to_base);
  transformed.header.stamp = pc->header.stamp;
  transformed.header.frame_id = "odom";
  camera_projection_pub_.publish(transformed);

  MapUtils::projectTo2D(transformed);

  octomapper_->insertPoints(camera_map_pair_, transformed, true, camera_probability_model_);

  octomapper_->get_updated_map(camera_map_pair_);
}

void Mapper::insertSegmentedImage(cv::Mat& image, const tf::Transform& odom_to_camera)
{
  PointCloud projected_pc;
  processImageFreeSpace(image);
  MapUtils::projectToPlane(projected_pc, ground_plane_, image, camera_model_, odom_to_camera);
  octomapper_->insertPoints(camera_map_pair_, projected_pc, false, camera_probability_model_);

  octomapper_->get_updated_map(camera_map_pair_);
}

void Mapper::processImageFreeSpace(cv::Mat& image) const
{
  cv::GaussianBlur(image, image, cv::Size(process_image_options_.blur.kernel, process_image_options_.blur.kernel),
                   process_image_options_.blur.sigma, process_image_options_.blur.sigma);
  cv::threshold(image, image, process_image_options_.threshold.threshold, UCHAR_MAX, cv::THRESH_BINARY);
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
