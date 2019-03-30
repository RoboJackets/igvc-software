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
#include <igvc_utils/NodeUtils.hpp>

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

  explicit Octomapper(ros::NodeHandle pNh);
  void create_octree(pc_map_pair& pair) const;
//  void insert_scan(const tf::Point &sensor_pos_tf, struct pc_map_pair &pc_map_pair,
//      const PCL_point_cloud &raw_pc, const pcl::PointCloud<pcl::PointXYZ> &empty_pc,
//      const GroundFilterOptions &ground_filter_options,
//      const ProbabilityModel &probability_model, GroundPlane &ground_plane);
//  void insertCameraProjection(struct pc_map_pair& projection_map_pair, const PCL_point_cloud& raw_pc, bool occupied) const;
//  void insert_camera_free(struct pc_map_pair& projection_map_pair, const cv::Mat& image,
//                          const image_geometry::PinholeCameraModel& model, const tf::Transform& camera_to_world) const;
  void get_updated_map(struct pc_map_pair& pc_map_pair) const;

  void insertScan(const tf::Point& sensor_pos, struct pc_map_pair& pair, const PointCloud& pc, ProbabilityModel model) const;
  void insertRays(const tf::Point& sensor_pos, struct pc_map_pair& pair, const PointCloud& pc, bool occupied, ProbabilityModel model) const;
  void insertPoints(struct pc_map_pair& pair, const PointCloud& pc, bool occupied, ProbabilityModel model) const;

private:
  void create_map(pc_map_pair& pair) const;
  void insert_free(const octomap::Pointcloud& scan, octomap::point3d origin, pc_map_pair& pair, bool lazy_eval) const;

  ros::NodeHandle pNh;
  bool m_use_ground_filter;
  double m_octree_resolution;
  double m_prob_hit;
  double m_prob_miss;
  double m_prob_miss_empty;
  double m_thresh_min;
  double m_thresh_max;
  double m_max_range;
  double m_floor_thresh;

  // Camera free space blurring
  int m_segmented_kernel;
  int m_segmented_sigma;
  int m_segmented_threshold;

  Ground_plane m_ground_projection;
  Ground_plane m_default_projection;

  int m_ransac_iterations;
  int m_openmp_threads;
  double m_ransac_distance_threshold;
  double m_ransac_eps_angle;
  float m_ground_filter_plane_dist;
  int m_map_length;
  int m_map_width;
  int m_map_encoding;
  float m_odds_sum_default;

  ros::Publisher m_debug_pub;

  // std::unique_ptr<tf::MessageFilter<PCL_point_cloud>> msg_filter;
};

#endif  // PROJECT_OCTOMAPPER_H
