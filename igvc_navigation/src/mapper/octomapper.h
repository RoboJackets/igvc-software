/**
 * Class that interfaces with OctoMap for efficient storage of 2D and 3D geometrical data. Contains convenience methods
 * for insertion of lidar scans, rays and points. Also provides a method to convert back from the OctoMap to a 2d array
 * in the form of a cv::Mat.
 *
 * Author: Oswin So <oswinso@gatech.edu>
 * Date Created: March 24 2019
 */
#ifndef PROJECT_OCTOMAPPER_H
#define PROJECT_OCTOMAPPER_H

#include <ros/ros.h>

#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>

#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <boost/shared_ptr.hpp>

#include <igvc_msgs/map.h>
#include <image_geometry/pinhole_camera_model.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "map_utils.h"

struct pc_map_pair
{
  boost::shared_ptr<octomap::OcTree> octree;
  boost::shared_ptr<cv::Mat> map;
};

struct ProbabilityModel
{
  double prob_miss;
  double prob_hit;
};

struct OcTreeOptions
{
  double resolution;
  double max;
  double min;
};

struct Ray
{
  pcl::PointXYZ start;
  pcl::PointXYZ end;
};

/**
 * Struct representing options for the map.
 * Length and Width are both in m, not grid cells.
 */
struct MapOptions
{
  double length;
  double width;
  double start_x;
  double start_y;
  double default_logodds;
  double resolution;

  double lengthGrid() const
  {
    return length / resolution;
  }

  double widthGrid() const
  {
    return width / resolution;
  }
};

/**
 * Struct representing the coefficients of the plane Ax + By + Cz + D = 0
 */
struct Ground_plane
{
  float a;
  float b;
  float c;
  float d;
  bool is_defined;

public:
  void set(const std::vector<float>& coeff)
  {
    a = coeff.at(0);
    b = coeff.at(1);
    c = coeff.at(2);
    d = coeff.at(3);
    is_defined = true;
  }
};

using radians = double;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

class Octomapper
{
public:
  using PCL_point_cloud = pcl::PointCloud<pcl::PointXYZ>;

  explicit Octomapper(const ros::NodeHandle& pNh);
  void create_octree(pc_map_pair& pair) const;

  /**
   * Updates the map to obtain the most updated version from the octree
   * @param pc_map_pair
   */
  void get_updated_map(struct pc_map_pair& pc_map_pair) const;

  /**
   * Inserts a lidar scan into the given pc_map_pair. This means that the points themselves are marked as occupied,
   * while the rays from the sensor_pos to the points are marked unoccupied.
   * @param sensor_pos
   * @param pair
   * @param pc
   * @param model
   * @param range
   */
  void insertScan(const tf::Point& sensor_pos, struct pc_map_pair& pair, const PointCloud& pc, ProbabilityModel model,
                  double range) const;

  /**
   * Inserts a ray into the given pc_map_pair, where the rays from the sensor_pos to the points are marked as either
   * occupied or not occupied, depending on the passed in variable
   * @param sensor_pos
   * @param pair
   * @param pc
   * @param occupied
   * @param model
   */
  void insertRays(const tf::Point& sensor_pos, struct pc_map_pair& pair, const PointCloud& pc, bool occupied,
                  ProbabilityModel model) const;

  /**
   * Inserts a ray into the given pc_map_pair, where each ray has a start and end position, with the points in the ray
   * marked as either occupied or not occupied, and the probabilities used depending on the ProbabilityModel passed in
   * @param pair
   * @param pc_rays
   * @param occupied
   * @param model
   */
  void insertRaysWithStartPoint(pc_map_pair& pair, const std::vector<Ray>& pc_rays, bool occupied,
                                ProbabilityModel model) const;

  /**
   * Inserts the passed in points to the given pc_map_pair, where the points are either occupied or not depending
   * on the variable passed in
   * @param pair
   * @param pc
   * @param occupied
   * @param model
   */
  void insertPoints(struct pc_map_pair& pair, const PointCloud& pc, bool occupied, ProbabilityModel model) const;

  /**
   * Inserts the passed in occupied and free points to the given pc_map_pair
   * @param pair
   * @param occupied_pc
   * @param free_pc
   * @param model
   */
  void insertPoints(struct pc_map_pair& pair, const PointCloud& occupied_pc, const PointCloud& free_pc,
                    ProbabilityModel model) const;

private:
  void create_map(pc_map_pair& pair) const;
  void insert_free(const octomap::Pointcloud& scan, octomap::point3d origin, pc_map_pair& pair, bool lazy_eval) const;
  std::pair<int, int> toMapCoordinates(double x, double y) const;

  ros::NodeHandle pNh;
  OcTreeOptions octree_options_;
  MapOptions map_options_;

  int map_encoding_;

  // std::unique_ptr<tf::MessageFilter<PCL_point_cloud>> msg_filter;
};

#endif  // PROJECT_OCTOMAPPER_H
