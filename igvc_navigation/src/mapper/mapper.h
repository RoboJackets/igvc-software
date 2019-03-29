#ifndef SRC_MAPPER_H
#define SRC_MAPPER_H

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <ros/publisher.h>
#include "octomapper.h"

class Mapper {
public:
  Mapper();

  void insertLidarScan(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc);
  void insertCameraProjection(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc);
  void insertSegmentedImage(const sensor_msgs::ImageConstPtr& image);

  void setProjectionModel(const image_geometry::PinholeCameraModel&& camera_model_);

  std::shared_ptr<cv::Mat> getMap();
private:
  std::unique_ptr<Octomapper> octomapper_;
  pc_map_pair pc_map_pair_;      // Struct storing both the octomap for the lidar and the cv::Mat map
  pc_map_pair camera_map_pair_;  // Struct storing both the octomap for the camera projections and the cv::Mat map
};

#endif //SRC_MAPPER_H
