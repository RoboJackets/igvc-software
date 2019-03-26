/**
 * Class containing a variety of filtering functions
 *
 * Author: Oswin So <oswinso@gatech.edu>
 * Date Created: March 24 2019
 */
#ifndef PROJECT_MAP_FILTERS_H
#define PROJECT_MAP_FILTERS_H

#include <optional>

#include <opencv2/opencv.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

struct EmptyFilterOptions
{
  using radians = double;

  radians start_angle;
  radians end_angle;
  double miss_cast_distance;
};

struct BehindFilterOptions
{
  using radians = double;

  radians angle;
  double distance;
};

struct RANSACOptions
{
  int iterations;
  double distance_threshold;
  double eps_angle;
};

struct FallbackOptions
{
  float plane_distance;
};

struct GroundFilterOptions
{
  RANSACOptions ransac_options;
  FallbackOptions fallback_options;
};

struct GroundPlane
{
  double a;
  double b;
  double c;
  double d;
};

namespace MapUtils
{
using radians = double;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

/**
 * Discretizes angle to ints according to the angular resolution of the lidar
 * @param[in] angle the undescritized angle
 * @param[in] angular_resolution the angular resolution to discretize with
 * @return
 */
int discretize(radians angle, double angular_resolution);

/**
 * Returns a pcl::PointCloud of points which have not been detected by the lidar scan, according to the angular
 * resolution
 * @param[in] pc lidar scan
 * @param[out] empty_pc points that are found to be free
 * @param[in] angular_resolution angular resolution to use in filter
 * @param[in] options Parameters to use for the filter
 */
void getEmptyPoints(const PointCloud& pc, PointCloud& empty_pc, double angular_resolution, EmptyFilterOptions options);

/**
 * Filters points behind the robot
 * @param[in] pc lidar scan
 * @param[out] filtered_pc filtered pointcloud
 */

void filterPointsBehind(const PointCloud& pc, PointCloud& filtered_pc, BehindFilterOptions options);

/**
 * Filters the ground plane by trying RANSAC first, then resorting to a z-coordinate based fallback filter.
 * @param raw_pc the unfiltered pointcloud
 * @param ground pointcloud representing the ground
 * @param nonground filtered pointcloud without the ground
 * @param options options for the filter
 * @return a std::optional containing the coefficients of the detected ground plane if RANSAC is used, otherwise
 * std::nullopt
 */
std::optional<GroundPlane> filterGroundPlane(const PointCloud& raw_pc, PointCloud& ground, PointCloud& nonground,
                                             GroundFilterOptions options);

/**
 * Attempts to filter the ground plane using RANSAC.
 * @param raw_pc the unfiltered pointcloud
 * @param ground pointcloud representing the ground
 * @param nonground filtered pointcloud without the ground
 * @param options options for the filter
 * @return a std::optional containing the coefficients of the detected ground plane if RANSAC is used, otherwise
 * std::nullopt
 */
std::optional<GroundPlane> ransacFilter(const PointCloud& raw_pc, PointCloud& ground, PointCloud& nonground,
                                        RANSACOptions options);

/**
 * Filters the ground plane using the z coordinate
 * @param raw_pc the unfiltered pointcloud
 * @param ground pointcloud representing the ground
 * @param nonground filtered pointcloud without the ground
 * @param options options for the filter
 */
void fallbackFilter(const PointCloud& raw_pc, PointCloud& ground, PointCloud& nonground, FallbackOptions options);

/**
 * Projects all black pixels (0, 0, 0) in the image to the ground plane and inserts them into the pointcloud
 * @param[out] projected_pc the pointcloud that holds all projected points
 * @param[in] ground_plane the coefficients of the ground plane
 * @param[in] image the image to project from
 * @param[in] model the camera model to be used for projection
 * @param[in] camera_to_world the tf::Transform from the camera to the world
 */
void projectToPlane(PointCloud& projected_pc, const GroundPlane& ground_plane, const cv::Mat& image,
                    const image_geometry::PinholeCameraModel& model, const tf::Transform& camera_to_world);

/**
 * Applied a gaussian blur with a kernel size of kernel_size
 * @param[in/out] blurred_map The map to be blurred
 * @param[in] kernel_size The kernel size to be applied
 */
void blur(cv::Mat& blurred_map, double kernel_size);
};  // namespace MapUtils

#endif  // PROJECT_MAP_FILTERS_H
