#ifndef SRC_MAPPER_H
#define SRC_MAPPER_H

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <ros/publisher.h>
#include "octomapper.h"
#include "map_utils.h"

class Mapper {
  using radians = double;
public:
  Mapper(const ros::NodeHandle& pNh);

  void insertLidarScan(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc, const tf::Transform& odom_to_lidar);
  void insertCameraProjection(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc, const tf::Transform& odom_to_base);
  void insertSegmentedImage(const cv::Mat& image, const tf::Transform& odom_to_camera);

  void setProjectionModel(const image_geometry::PinholeCameraModel&& camera_model_);

  std::shared_ptr<cv::Mat> getMap();
private:
  std::unique_ptr<Octomapper> octomapper_;
  pc_map_pair pc_map_pair_;      // Struct storing both the octomap for the lidar and the cv::Mat map
  pc_map_pair camera_map_pair_;  // Struct storing both the octomap for the camera projections and the cv::Mat map

  image_geometry::PinholeCameraModel camera_model_;

  EmptyFilterOptions empty_filter_options_{};
  BehindFilterOptions behind_filter_options_{};

  ros::Publisher camera_projection_pub;                              // Publishes blurred map

  bool camera_model_initialized_;
  radians angular_resolution_;

  double resolution_; // Map Resolution
  double radius_;  // Radius to filter lidar points

};

#endif //SRC_MAPPER_H
