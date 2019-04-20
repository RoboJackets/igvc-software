#include "mapper.h"
#include <pcl_ros/transforms.h>
#include "map_utils.h"

Mapper::Mapper(ros::NodeHandle& pNh) : ground_plane_{ 0, 0, 1, 0 }
{
  igvc::getParam(pNh, "sensor_model/max_range", radius_);
  igvc::getParam(pNh, "sensor_model/angular_resolution", angular_resolution_);

  igvc::getParam(pNh, "probability_model/lidar/scan/prob_miss", lidar_scan_probability_model_.prob_miss);
  igvc::getParam(pNh, "probability_model/lidar/scan/prob_hit", lidar_scan_probability_model_.prob_hit);

  igvc::getParam(pNh, "probability_model/lidar/ground/prob_miss", lidar_ground_probability_model_.prob_miss);

  igvc::getParam(pNh, "probability_model/lidar/free_space/prob_miss", lidar_free_space_probability_model_.prob_miss);

  igvc::getParam(pNh, "probability_model/camera/prob_miss", camera_probability_model_.prob_miss);
  igvc::getParam(pNh, "probability_model/camera/prob_hit", camera_probability_model_.prob_hit);

  igvc::getParam(pNh, "filters/ground_filter/enable", use_ground_filter_);
  igvc::getParam(pNh, "filters/ground_filter/ransac/iterations", ground_filter_options_.ransac_options.iterations);
  igvc::getParam(pNh, "filters/ground_filter/ransac/iterations", ground_filter_options_.ransac_options.iterations);
  igvc::getParam(pNh, "filters/ground_filter/ransac/distance_threshold",
                 ground_filter_options_.ransac_options.distance_threshold);
  igvc::getParam(pNh, "filters/ground_filter/ransac/eps_angle", ground_filter_options_.ransac_options.eps_angle);
  igvc::getParam(pNh, "filters/ground_filter/fallback/plane_distance",
                 ground_filter_options_.fallback_options.plane_distance);

  igvc::param(pNh, "filters/empty/enabled", empty_filter_options_.enabled, false);
  igvc::getParam(pNh, "filters/empty/start_angle", empty_filter_options_.start_angle);
  igvc::getParam(pNh, "filters/empty/end_angle", empty_filter_options_.end_angle);
  igvc::getParam(pNh, "filters/empty/miss_cast_distance", empty_filter_options_.miss_cast_distance);
  igvc::getParam(pNh, "filters/empty/max_range", empty_filter_options_.max_range);

  igvc::getParam(pNh, "filters/empty_image/blur/kernel", process_image_options_.blur.kernel);
  igvc::getParam(pNh, "filters/empty_image/blur/sigma", process_image_options_.blur.sigma);
  igvc::getParam(pNh, "filters/empty_image/threshold", process_image_options_.threshold.threshold);
  igvc::getParam(pNh, "filters/empty_image/dilation_size", process_image_options_.dilation_size);

  igvc::param(pNh, "filters/behind/enabled", behind_filter_options_.enabled, false);
  igvc::getParam(pNh, "filters/behind/filter_angle", behind_filter_options_.angle);
  igvc::getParam(pNh, "filters/behind/distance", behind_filter_options_.distance);

  igvc::getParam(pNh, "filters/combined_map/blur/kernel", combined_map_options_.blur.kernel);

  igvc::getParam(pNh, "octree/resolution", resolution_);

  igvc::getParam(pNh, "node/use_lines", use_lines_);
  igvc::getParam(pNh, "node/debug", debug_);

  invertMissProbabilities();

  octomapper_ = std::make_unique<Octomapper>(pNh);
  octomapper_->create_octree(pc_map_pair_);
  octomapper_->create_octree(camera_map_pair_);

  camera_line_pub_ = pNh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/mapper/camera_lines", 1);

  camera_projection_pub_left_ = pNh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/mapper/camera_projection_left", 1);
  camera_projection_pub_center_ = pNh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/mapper/camera_projection_center", 1);
  camera_projection_pub_right_ = pNh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/mapper/camera_projection_right", 1);

  filtered_pc_pub_ = pNh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/mapper/filtered_pc", 1);
  empty_pc_pub_ = pNh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/mapper/empty_pc", 1);
  ground_pub_ = pNh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/mapper/ground", 1);
  nonground_pub_ = pNh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/mapper/nonground", 1);
  nonground_projected_pub_ = pNh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/mapper/nonground_projected", 1);
}

void Mapper::insertLidarScan(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc, const tf::Transform& lidar_to_odom)
{
  pcl::PointCloud<pcl::PointXYZ> empty_pc{};
  pcl::PointCloud<pcl::PointXYZ> filtered_pc{};

  if (behind_filter_options_.enabled)
  {
    MapUtils::filterPointsBehind(*pc, filtered_pc, behind_filter_options_);
  }
  else
  {
    filtered_pc = *pc;
  }

  pcl_ros::transformPointCloud(filtered_pc, filtered_pc, lidar_to_odom);

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

    MapUtils::debugPublishPointCloud(nonground_pub_, nonground, pc->header.stamp, "/odom", debug_);
    MapUtils::debugPublishPointCloud(ground_pub_, ground, pc->header.stamp, "/odom", debug_);

    MapUtils::projectTo2D(nonground);
    MapUtils::debugPublishPointCloud(nonground_projected_pub_, nonground, pc->header.stamp, "/odom", debug_);

    // Transform to lidar, filter empty, then transform back
    if (empty_filter_options_.enabled)
    {
      pcl::PointCloud<pcl::PointXYZ> nonground_lidar;
      pcl_ros::transformPointCloud(nonground, nonground_lidar, lidar_to_odom.inverse());

      MapUtils::getEmptyPoints(nonground_lidar, empty_pc, angular_resolution_, empty_filter_options_);
      pcl_ros::transformPointCloud(empty_pc, empty_pc, lidar_to_odom);
      MapUtils::projectTo2D(empty_pc);

      MapUtils::debugPublishPointCloud(empty_pc_pub_, empty_pc, pc->header.stamp, "/odom", debug_);
    }

    octomapper_->insertScan(lidar_to_odom.getOrigin(), pc_map_pair_, nonground, lidar_scan_probability_model_, radius_);
  }
  else
  {
    MapUtils::projectTo2D(filtered_pc);
    octomapper_->insertScan(lidar_to_odom.getOrigin(), pc_map_pair_, filtered_pc, lidar_scan_probability_model_,
                            radius_);

    if (empty_filter_options_.enabled)
    {
      MapUtils::getEmptyPoints(*pc, empty_pc, angular_resolution_, empty_filter_options_);
      pcl_ros::transformPointCloud(empty_pc, empty_pc, lidar_to_odom);

      MapUtils::projectTo2D(empty_pc);
      MapUtils::debugPublishPointCloud(empty_pc_pub_, empty_pc, pc->header.stamp, "/odom", debug_);
    }
  }
  octomapper_->insertRays(lidar_to_odom.getOrigin(), pc_map_pair_, empty_pc, false,
                          lidar_free_space_probability_model_);

  octomapper_->get_updated_map(pc_map_pair_);
}

void Mapper::insertCameraProjection(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc,
                                    const tf::Transform& base_to_odom)
{
  pcl::PointCloud<pcl::PointXYZ> transformed{};

  pcl_ros::transformPointCloud(*pc, transformed, base_to_odom);
  MapUtils::debugPublishPointCloud(camera_line_pub_, transformed, pc->header.stamp, "/odom", debug_);

  MapUtils::projectTo2D(transformed);

  octomapper_->insertPoints(camera_map_pair_, transformed, true, camera_probability_model_);

  octomapper_->get_updated_map(camera_map_pair_);
}

void Mapper::insertSegmentedImage(cv::Mat&& image, const tf::Transform& base_to_odom,
                                  const tf::Transform& camera_to_base, const ros::Time& stamp, Camera camera)
{
  if (camera_model_map_.find(camera) == camera_model_map_.end() && !camera_model_initialized_ && !use_ground_filter_)
  {
    ROS_WARN_STREAM_THROTTLE(1, "camera model not initialized or not using ground filter");
    return;
  }

  PointCloud projected_pc;
  processImageFreeSpace(image);

  image_geometry::PinholeCameraModel model = camera_model_map_.at(camera);

  MapUtils::projectToPlane(projected_pc, ground_plane_, image, model, camera_to_base);

  pcl_ros::transformPointCloud(projected_pc, projected_pc, base_to_odom);
  MapUtils::projectTo2D(projected_pc);
  if (static_cast<int>(camera) == 0)
  {
    MapUtils::debugPublishPointCloud(camera_projection_pub_left_, projected_pc, pcl_conversions::toPCL(stamp), "/odom",
                                     debug_);
  }
  else if (static_cast<int>(camera) == 1)
  {
    MapUtils::debugPublishPointCloud(camera_projection_pub_center_, projected_pc, pcl_conversions::toPCL(stamp),
                                     "/odom", debug_);
  }
  else
  {
    MapUtils::debugPublishPointCloud(camera_projection_pub_right_, projected_pc, pcl_conversions::toPCL(stamp), "/odom",
                                     debug_);
  }

  octomapper_->insertPoints(camera_map_pair_, projected_pc, false, camera_probability_model_);

  octomapper_->get_updated_map(camera_map_pair_);
}

void Mapper::processImageFreeSpace(cv::Mat& image) const
{
  cv::GaussianBlur(image, image, cv::Size(process_image_options_.blur.kernel, process_image_options_.blur.kernel),
                   process_image_options_.blur.sigma, process_image_options_.blur.sigma);
  cv::threshold(image, image, process_image_options_.threshold.threshold, UCHAR_MAX, cv::THRESH_BINARY);
  cv::Mat element = cv::getStructuringElement(
      cv::MORPH_RECT,
      cv::Size(2 * process_image_options_.dilation_size + 1, 2 * process_image_options_.dilation_size + 1),
      cv::Point(process_image_options_.dilation_size, process_image_options_.dilation_size));
  cv::dilate(image, image, element);
}

std::optional<cv::Mat> Mapper::getMap()
{
  if (!pc_map_pair_.map)
  {
    ROS_WARN_STREAM_THROTTLE(1, "pc_map_pair is null...");
    return std::nullopt;
  }

  cv::Mat blurred_map;
  if (use_lines_)
  {
    if (!camera_map_pair_.map)
    {
      ROS_WARN_STREAM_THROTTLE(1, "camera_map_pair is null...");
      return std::nullopt;
    }
    blurred_map = pc_map_pair_.map->clone();
    cv::max(blurred_map, *camera_map_pair_.map, blurred_map);
  }
  else
  {
    blurred_map = pc_map_pair_.map->clone();
  }
  MapUtils::blur(blurred_map, combined_map_options_.blur.kernel);
  return blurred_map;
}

void Mapper::setProjectionModel(const image_geometry::PinholeCameraModel& camera_model, Camera camera)
{
  camera_model_map_.insert(std::make_pair(camera, camera_model));
  ROS_INFO_STREAM_THROTTLE(1, "Projection Model set for camera " << static_cast<int>(camera));
}

/**
 * Invert prob_miss of probability models so that configuration in the yaml is more intuitive
 */
void Mapper::invertMissProbabilities()
{
  lidar_free_space_probability_model_.prob_miss = 1.0 - lidar_free_space_probability_model_.prob_miss;
  lidar_scan_probability_model_.prob_miss = 1.0 - lidar_scan_probability_model_.prob_miss;
  lidar_ground_probability_model_.prob_miss = 1.0 - lidar_ground_probability_model_.prob_miss;
  camera_probability_model_.prob_miss = 1.0 - camera_probability_model_.prob_miss;
}
