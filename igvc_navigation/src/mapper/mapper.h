#ifndef SRC_MAPPER_H
#define SRC_MAPPER_H

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <ros/publisher.h>
#include "map_utils.h"
#include "octomapper.h"

struct BlurFilterOptions
{
  int kernel;
  double sigma;
};

struct ThresholdFilterOptions
{
  double threshold;
};

struct ProcessImageOptions
{
  BlurFilterOptions blur;
  ThresholdFilterOptions threshold;
};

struct CombinedMapOptions
{
  BlurFilterOptions blur;
};

class Mapper
{
  using radians = double;

public:
  Mapper(ros::NodeHandle& pNh);

  /**
   * Inserts a lidar scan into the map.
   * @param pc
   * @param odom_to_lidar
   */
  void insertLidarScan(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc, const tf::Transform& odom_to_lidar);

  /**
   * Inserts a pointcloud containing the projection of lines.
   * @param pc
   * @param odom_to_base
   */
  void insertCameraProjection(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc, const tf::Transform& odom_to_base);

  /**
   * Projects the empty space in the passed in image and inserts it into the map.
   * @param image
   * @param odom_to_base
   * @param base_to_camera
   * @param stamp
   */
  void insertSegmentedImage(cv::Mat& image, const tf::Transform& odom_to_base, const tf::Transform& base_to_camera,
                            const ros::Time& stamp);

  /**
   * Sets the parameters for the image_geometry::PinholeCameraModel used for projection.
   * @param camera_model_
   */
  void setProjectionModel(image_geometry::PinholeCameraModel camera_model_);

  /**
   * Returns the current map, or std::nullopt if no data has been receieved yet.ction Tool_External Tools_clang-format
   *
   * @return
   */
  std::optional<cv::Mat> getMap();

private:
  /**
   * Performs filtering on the free space of an image
   * @param[in/out] image the image to be filtered
   */
  void processImageFreeSpace(cv::Mat& image) const;

  std::unique_ptr<Octomapper> octomapper_;
  pc_map_pair pc_map_pair_;      // Struct storing both the octomap for the lidar and the cv::Mat map
  pc_map_pair camera_map_pair_;  // Struct storing both the octomap for the camera projections and the cv::Mat map

  image_geometry::PinholeCameraModel camera_model_;

  EmptyFilterOptions empty_filter_options_{};
  BehindFilterOptions behind_filter_options_{};
  ProcessImageOptions process_image_options_{};
  CombinedMapOptions combined_map_options_{};

  ros::Publisher camera_projection_pub_;
  ros::Publisher camera_projection_pub2_;
  ros::Publisher camera_line_pub_;

  ProbabilityModel lidar_scan_probability_model_{};
  ProbabilityModel lidar_ground_probability_model_{};
  ProbabilityModel lidar_free_space_probability_model_{};

  ProbabilityModel camera_probability_model_{};
  GroundFilterOptions ground_filter_options_{};
  GroundPlane ground_plane_;

  bool use_ground_filter_;
  bool camera_model_initialized_;
  bool use_lines_;
  radians angular_resolution_;

  double resolution_;  // Map Resolution
  double radius_;      // Radius to filter lidar points
  double combined_blur_kernel_size_;
};

#endif  // SRC_MAPPER_H
