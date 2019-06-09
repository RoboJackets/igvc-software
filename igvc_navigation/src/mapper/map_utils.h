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
#include <tf/transform_datatypes.h>

struct EmptyFilterOptions
{
  using radians = double;

  bool enabled;
  radians start_angle;
  radians end_angle;
  double miss_cast_distance;
  double max_range;
};

struct BehindFilterOptions
{
  using radians = double;

  bool enabled;
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

struct RemoveOccupiedOptions
{
  bool enable;
  int kernel_size;
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
 * @return the index of the angle in increments of angular_resolution
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
 * @param[in] raw_pc the unfiltered pointcloud
 * @param[out] ground pointcloud representing the ground
 * @param[out] nonground filtered pointcloud without the ground
 * @param[in] options options for the filter
 * @return a std::optional containing the coefficients of the detected ground plane if RANSAC is used, otherwise
 * std::nullopt
 */
std::optional<GroundPlane> filterGroundPlane(const PointCloud& raw_pc, PointCloud& ground, PointCloud& nonground,
                                             GroundFilterOptions options);

/**
 * Attempts to filter the ground plane using RANSAC.
 * @param[in] raw_pc the unfiltered pointcloud
 * @param[out] ground pointcloud representing the ground
 * @param[out] nonground filtered pointcloud without the ground
 * @param[in] options options for the filter
 * @return a std::optional containing the coefficients of the detected ground plane if RANSAC is used, otherwise
 * std::nullopt
 */
std::optional<GroundPlane> ransacFilter(const PointCloud& raw_pc, PointCloud& ground, PointCloud& nonground,
                                        const RANSACOptions& options);

/**
 * Filters the ground plane using the z coordinate
 * @param[in] raw_pc the unfiltered pointcloud
 * @param[out] ground pointcloud representing the ground
 * @param[out] nonground filtered pointcloud without the ground
 * @param[in] options options for the filter
 */
void fallbackFilter(const PointCloud& raw_pc, PointCloud& ground, PointCloud& nonground,
                    const FallbackOptions& options);

void removeOccupiedFromImage(cv::Mat& image, const PointCloud& last_scan,
                             const image_geometry::PinholeCameraModel& camera_model,
                             const tf::Transform& camera_to_odom, const RemoveOccupiedOptions& options);

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
 * Projects all points in projected_pc to z=0
 * @param[in/out] projected_pc the pointcloud to project to z=0
 */
void projectTo2D(PointCloud& projected_pc);

/**
 * Applied a gaussian blur with a kernel size of kernel_size
 * @param[in/out] blurred_map The map to be blurred
 * @param[in] kernel_size The kernel size to be applied
 */
void blur(cv::Mat& blurred_map, double kernel_size);

/**
 * Scales the intrinsics in camera_info to a resized dimension width and height
 * @param camera_info original CameraInfo
 * @param width scaled width
 * @param height scaled height
 * @return the CameraInfo with scaled intrinsics
 */
sensor_msgs::CameraInfoConstPtr scaleCameraInfo(const sensor_msgs::CameraInfoConstPtr& camera_info, double width,
                                                double height);

inline bool withinRange(const pcl::PointXYZ& point, double range);

void debugPublishPointCloud(const ros::Publisher& publisher, pcl::PointCloud<pcl::PointXYZ>& pointcloud,
                            const uint64 stamp, std::string&& frame, bool debug);
void debugPublishImage(const ros::Publisher& publisher, const cv::Mat& image, const ros::Time stamp, bool debug);
}  // namespace MapUtils

inline bool MapUtils::withinRange(const pcl::PointXYZ& point, double range)
{
  return std::hypot(point.x, point.y, point.z) <= range;
}

#endif  // PROJECT_MAP_FILTERS_H
