#include "mapper.h"
#include <pcl_ros/transforms.h>
#include "map_utils.h"

Mapper::Mapper(ros::NodeHandle& pNh)
{
  igvc::getParam(pNh, "sensor_model/max_range", radius_);
  igvc::getParam(pNh, "sensor_model/angular_resolution", angular_resolution_);

  igvc::getParam(pNh, "probability_model/lidar_scan/prob_miss", lidar_probability_model_.prob_miss);
  igvc::getParam(pNh, "probability_model/lidar_scan/prob_hit", lidar_probability_model_.prob_hit);
  igvc::getParam(pNh, "probability_model/camera/prob_miss", camera_probability_model_.prob_miss);
  igvc::getParam(pNh, "probability_model/camera/prob_hit", camera_probability_model_.prob_hit);

  igvc::getParam(pNh, "filters/ground_filter/ransac/iterations", ground_filter_options_.ransac_options.iterations);
  igvc::getParam(pNh, "filters/ground_filter/ransac/distance_threshold", ground_filter_options_.ransac_options.distance_threshold);
  igvc::getParam(pNh, "filters/ground_filter/ransac/eps_angle", ground_filter_options_.ransac_options.eps_angle);
  igvc::getParam(pNh, "filters/ground_filter/fallback/plane_distance", ground_filter_options_.fallback_options.plane_distance);

  igvc::getParam(pNh, "filters/empty/start_angle", empty_filter_options_.start_angle);
  igvc::getParam(pNh, "filters/empty/end_angle", empty_filter_options_.end_angle);
  igvc::getParam(pNh, "filters/empty/miss_cast_distance", empty_filter_options_.miss_cast_distance);

  igvc::getParam(pNh, "filters/empty_image/blur/kernel", process_image_options_.blur.kernel);
  igvc::getParam(pNh, "filters/empty_image/blur/sigma", process_image_options_.blur.sigma);
  igvc::getParam(pNh, "filters/empty_image/threshold", process_image_options_.threshold.threshold);

  igvc::getParam(pNh, "filters/behind/filter_angle", behind_filter_options_.angle);
  igvc::getParam(pNh, "filters/behind/distance", behind_filter_options_.distance);

  igvc::getParam(pNh, "filters/combined_map/blur/kernel", combined_map_options_.blur.kernel);

  igvc::getParam(pNh, "octree/resolution", resolution_);

  igvc::getParam(pNh, "node/use_lines", use_lines_);

  octomapper_ = std::make_unique<Octomapper>(pNh);
  octomapper_->create_octree(pc_map_pair_);
  octomapper_->create_octree(camera_map_pair_);

  camera_projection_pub_ = pNh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/mapper/camera_projection", 1);
}

void Mapper::insertLidarScan(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc, const tf::Transform& odom_to_lidar)
{
  ROS_INFO_STREAM_THROTTLE(1, "insertLidarScan");
  pcl::PointCloud<pcl::PointXYZ> empty_pc{};
  pcl::PointCloud<pcl::PointXYZ> filtered_pc{};

  MapUtils::getEmptyPoints(*pc, empty_pc, angular_resolution_, empty_filter_options_);
  MapUtils::filterPointsBehind(*pc, filtered_pc, behind_filter_options_);

  pcl_ros::transformPointCloud(filtered_pc, filtered_pc, odom_to_lidar);
  pcl_ros::transformPointCloud(empty_pc, empty_pc, odom_to_lidar);

  filtered_pc.header.stamp = pc->header.stamp;
  filtered_pc.header.frame_id = "/odom";
  camera_projection_pub_.publish(filtered_pc);

  ROS_INFO_STREAM_THROTTLE(1, "finished transforming PointClouds.");
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
  ROS_INFO_STREAM_THROTTLE(1, "finished inserting rays.");

  octomapper_->get_updated_map(pc_map_pair_);
  ROS_INFO_STREAM_THROTTLE(1, "finished updating map");
}

void Mapper::insertCameraProjection(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc,
                                    const tf::Transform& odom_to_base)
{
  ROS_INFO_STREAM_THROTTLE(1, "insertCameraProjection");
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
  ROS_INFO_STREAM_THROTTLE(1, "insertSegmentedImage");
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

std::optional<cv::Mat> Mapper::getMap()
{
  ROS_INFO_STREAM_THROTTLE(1, "in getMap");

  if (!pc_map_pair_.map) {
    ROS_WARN_STREAM_THROTTLE(1, "pc_map_pair is null...");
    return std::nullopt;
  }

  cv::Mat blurred_map;
  if (use_lines_)
  {
    if (!camera_map_pair_.map) {
      ROS_WARN_STREAM_THROTTLE(1, "camera_map_pair is null...");
      return std::nullopt;
    }
    blurred_map = pc_map_pair_.map->clone();
    cv::max(blurred_map, *camera_map_pair_.map, blurred_map);
  } else {
    ROS_INFO_STREAM_THROTTLE(1, "here right?");
    ROS_INFO_STREAM("map width: " << pc_map_pair_.map->rows << ", height: " << pc_map_pair_.map->cols);
    blurred_map = pc_map_pair_.map->clone();
    MapUtils::blur(blurred_map, combined_map_options_.blur.kernel);
    ROS_INFO_STREAM_THROTTLE(1, "finished blurring");
  }
  return blurred_map;
}

void Mapper::setProjectionModel(const image_geometry::PinholeCameraModel&& camera_model)
{
  camera_model_ = camera_model;
  camera_model_initialized_ = true;
}
