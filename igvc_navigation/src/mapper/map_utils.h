/**
 * Class containing a variety of filtering functions
 *
 * Author: Oswin So <oswinso@gatech.edu>
 * Date Created: March 24 2019
 */
#ifndef PROJECT_MAP_FILTERS_H
#define PROJECT_MAP_FILTERS_H

#include <opencv2/opencv.hpp>

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

namespace MapUtils
{
using radians = double;

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
void getEmptyPoints(const pcl::PointCloud<pcl::PointXYZ> &pc, pcl::PointCloud<pcl::PointXYZ> &empty_pc,
                    double angular_resolution, EmptyFilterOptions options);

/**
 * Filters points behind the robot
 * @param[in] pc lidar scan
 * @param[out] filtered_pc filtered pointcloud
 */

void filterPointsBehind(const pcl::PointCloud<pcl::PointXYZ> &pc, pcl::PointCloud<pcl::PointXYZ> &filtered_pc, BehindFilterOptions options);

/**
 * Applied a gaussian blur with a kernel size of kernel_size
 * @param[in/out] blurred_map The map to be blurred
 * @param[in] kernel_size The kernel size to be applied
 */
void blur(cv::Mat &blurred_map, double kernel_size);
};  // namespace MapUtils

#endif  // PROJECT_MAP_FILTERS_H
