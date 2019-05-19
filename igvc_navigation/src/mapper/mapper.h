/**
 * Class that performs the mapping, taking in pointclouds and camera data, and calling the respective methods
 * in Octomapper that actually stores the map information. Also processes the obtained information with a
 * variety of filters to improve mapping accuracy.
 *
 * Author: Oswin So <oswinso@gatech.edu>
 * Date Created: March 24 2019
 */
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
  int dilation_size;
};

struct CombinedMapOptions
{
  BlurFilterOptions blur;
};

enum class Camera : int
{
  left,
  center,
  right
};

class Mapper
{
  using radians = double;

public:
  Mapper(ros::NodeHandle& pNh);

  /**
   * Inserts a lidar scan into the map.
   * @param[in] pc pointcloud to be inserted
   * @param[in] lidar_to_odom transformation to transform from lidar frame to odom frame
   */
  void insertLidarScan(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc, const tf::Transform& lidar_to_odom);

  /**
   * Inserts a pointcloud containing the projection of lines.
   * @param[in] pc pointcloud to be inserted
   * @param[in] base_to_odom transformation to transform from base frame to odom frame
   */
  void insertCameraProjection(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc, const tf::Transform& base_to_odom);

  /**
   * Projects the empty space in the passed in image and inserts it into the map.
   * @param[in] image image to project.
   * @param[in] base_to_odom transformation to transform from base frame to odom frame
   * @param[in] camera_to_base transformation to transform from camera frame to odom frame
   * @param[in] stamp timestamp to be used for debug publishing
   */
  void insertSegmentedImage(cv::Mat&& image, const tf::Transform& base_to_odom, const tf::Transform& camera_to_base,
                            const ros::Time& stamp, Camera camera);

  /**
   * Sets the parameters for the image_geometry::PinholeCameraModel used for projection.
   * @param[in] camera_model camera model used for projection
   */
  void setProjectionModel(const image_geometry::PinholeCameraModel& camera_model, Camera camera);

  /**
   * Returns the current map, or std::nullopt if no data has been receieved yet.ction Tool_External Tools_clang-format
   * @return std::optional of the map as a cv::Mat
   */
  std::optional<cv::Mat> getMap();

private:
  /**
   * Performs filtering on the free space of an image
   * @param[in/out] image the image to be filtered
   */
  void processImageFreeSpace(cv::Mat& image) const;

  /**
   * Inverts probabilities for the ProbabilityModels so that miss_probability is more intuitive.
   */
  void invertMissProbabilities();

  std::unique_ptr<Octomapper> octomapper_;
  pc_map_pair pc_map_pair_;      // Struct storing both the octomap for the lidar and the cv::Mat map
  pc_map_pair camera_map_pair_;  // Struct storing both the octomap for the camera projections and the cv::Mat map

  std::unordered_map<Camera, image_geometry::PinholeCameraModel> camera_model_map_;

  EmptyFilterOptions empty_filter_options_{};
  BehindFilterOptions behind_filter_options_{};
  ProcessImageOptions process_image_options_{};
  CombinedMapOptions combined_map_options_{};

  ros::Publisher camera_projection_pub_left_;
  ros::Publisher camera_projection_pub_center_;
  ros::Publisher camera_projection_pub_right_;

  ros::Publisher filtered_pc_pub_;
  ros::Publisher empty_pc_pub_;
  ros::Publisher ground_pub_;
  ros::Publisher nonground_pub_;
  ros::Publisher nonground_projected_pub_;
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

  bool debug_pub_camera_lines;
  bool debug_pub_camera_projections;
  bool debug_pub_filtered_pointclouds;

  radians angular_resolution_;

  double resolution_;  // Map Resolution
  double radius_;      // Radius to filter lidar points
  double combined_blur_kernel_size_;
};

#endif  // SRC_MAPPER_H
