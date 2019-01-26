#pragma once
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
#include <igvc_utils/NodeUtils.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

struct pc_map_pair
{
  std::unique_ptr<octomap::OcTree> octree;
  std::unique_ptr<cv::Mat> map;
};

class Octomapper
{
public:
  using PCL_point_cloud = pcl::PointCloud<pcl::PointXYZ>;

  explicit Octomapper(ros::NodeHandle pNh);
  void create_octree(pc_map_pair& pair) const;
  void insert_scan(pc_map_pair& pc_map_pair, octomap::KeySet& free_cells, octomap::KeySet& occupied_cells) const;
  void get_updated_map(pc_map_pair& pc_map_pair) const;
  void filter_ground_plane(const PCL_point_cloud& pc, PCL_point_cloud& ground, PCL_point_cloud& nonground, const pcl::ModelCoefficientsPtr& coefficients) const;
  void separate_occupied(octomap::KeySet& free_cells, octomap::KeySet& occupied_cells, const tf::Point& sensor_pos_tf,
                         const pc_map_pair& pair, const pcl::PointCloud<pcl::PointXYZ>& ground,
                         const pcl::PointCloud<pcl::PointXYZ>& nonground);
  float sensor_model(const pc_map_pair& pair, const octomap::KeySet& free_cells,
                     const octomap::KeySet& occupied_cells) const;
  inline float to_logodds(float p) const
  {
    return log(p / (1 - p));
  }

  inline float from_logodds(float logodd) const
  {
    return (1 - 1/(1 + exp(logodd)));
  }
  int length_x() { return m_map_length_grid; }
  int width_y() { return m_map_width_grid; }
  int start_x() { return m_map_start_x; }
  int start_y() { return m_map_start_y; }
  double resolution() { return m_octree_resolution; } // TODO: Different resolution for octree / map?

private:
  void create_map(pc_map_pair& pair) const;
  void compute_voxels(const octomap::OcTree& tree, const octomap::Pointcloud& scan, const octomap::Pointcloud& ground,
                      const octomap::point3d& origin, octomap::KeySet& free_cells,
                      octomap::KeySet& occupied_cells);

  ros::NodeHandle pNh;
  double m_octree_resolution;
  bool m_lazy_eval;
  double m_prob_hit;
  double m_prob_miss;
  double m_prob_hit_logodds;
  double m_prob_miss_logodds;
  double m_sensor_model_occ_coeff, m_sensor_model_free_coeff;
  double m_penalty;
  double m_sensor_empty_coeff;
  double m_thresh_min;
  double m_thresh_max;
  double m_max_range;
  int m_sensor_model = 1;
  int m_ransac_iterations;
  double m_ransac_distance_threshold;
  double m_ransac_eps_angle;
  float m_ground_filter_plane_dist;
  int m_map_length; // In m, not grid units
  int m_map_width; // Actual grid cells = length / resolution
  int m_map_length_grid;
  int m_map_width_grid;
  int m_map_start_x; // In m, not grid units
  int m_map_start_y;
  int m_map_encoding;
  float m_odds_sum_default;
  ros::Publisher m_octo_viz_pub;
  std::vector<octomap::KeyRay> m_keyrays;

  // std::unique_ptr<tf::MessageFilter<PCL_point_cloud>> msg_filter;
};

#endif  // PROJECT_OCTOMAPPER_H
