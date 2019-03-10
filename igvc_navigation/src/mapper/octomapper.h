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
  boost::shared_ptr<octomap::OcTree> octree;
  boost::shared_ptr<cv::Mat> map;
};

class Octomapper
{
  using radians = double;

public:
  using PCL_point_cloud = pcl::PointCloud<pcl::PointXYZ>;

  explicit Octomapper(ros::NodeHandle pNh);
  void create_octree(pc_map_pair& pair) const;
  void insert_scan(const tf::Point& sensor_pos_tf, struct pc_map_pair& pc_map_pair, const PCL_point_cloud& raw_pc,
                   const pcl::PointCloud<pcl::PointXYZ>& empty_pc) const;
  void insert_camera_projection(struct pc_map_pair& pc_map_pair, const PCL_point_cloud& raw_pc) const;
  void get_updated_map(struct pc_map_pair& pc_map_pair) const;
  void filter_ground_plane(const PCL_point_cloud& pc, PCL_point_cloud& ground, PCL_point_cloud& nonground) const;

private:
  void filter_range(const pcl::PointCloud<pcl::PointXYZ>& raw_pc, pcl::PointCloud<pcl::PointXYZ>& within_range) const;
  void create_map(pc_map_pair& pair) const;
  void insert_free(const octomap::Pointcloud& scan, octomap::point3d origin, pc_map_pair& pair, bool lazy_eval) const;
  inline int discretize(radians angle) const;

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

  int m_ransac_iterations;
  int m_openmp_threads;
  double m_ransac_distance_threshold;
  double m_ransac_eps_angle;
  float m_ground_filter_plane_dist;
  int m_map_length;
  int m_map_width;
  int m_map_encoding;
  float m_odds_sum_default;

  // std::unique_ptr<tf::MessageFilter<PCL_point_cloud>> msg_filter;
};

#endif  // PROJECT_OCTOMAPPER_H
