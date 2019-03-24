#ifndef PROJECT_MAPPER_H
#define PROJECT_MAPPER_H

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <ros/publisher.h>
#include "octomapper.h"


class Mapper
{
  using radians = double;

public:
  Mapper();

private:
  // Callbacks
  void pcCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc);

  void segmentedImageCallback(const sensor_msgs::ImageConstPtr& segmented);

  void projctedLineCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc);

  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info);

  void publish(uint64_t stamp);

  void setMessageMetadata(igvc_msgs::map &message, sensor_msgs::Image &image, uint64_t pcl_stamp);

  template <class T>
  bool checkExistsStaticTransform(const std::string &frame_id, T timestamp, const std::string &topic);

  template <class T>
  bool getOdomTransform(const T stamp);

  sensor_msgs::CameraInfoConstPtr scaleCameraInfo(const sensor_msgs::CameraInfoConstPtr &camera_info);

  void publishAsPCL(const ros::Publisher& pub, const cv::Mat& mat, const std::string& frame_id, uint64_t stamp);

  cv_bridge::CvImage img_bridge_;

  ros::Publisher map_pub_;                                  // Publishes map
  ros::Publisher blurred_pub_;                              // Publishes blurred map
  ros::Publisher debug_pub_;                                // Debug version of above
  ros::Publisher debug_pcl_pub_;                            // Publishes map as individual PCL points
  ros::Publisher debug_blurred_pc_;                         // Publishes blurred map as individual PCL points
  ros::Publisher ground_pub_;                               // Publishes ground points
  ros::Publisher nonground_pub_;                            // Publishes non ground points
  ros::Publisher random_pub_;                               // Publisher for debugging pointcloud related things
  std::map<std::string, tf::StampedTransform> transforms_;  // Map of static transforms TODO: Refactor this
  std::unique_ptr<tf::TransformListener> tf_listener_;      // TF Listener

  bool use_lines_;
  bool camera_model_initialized_{ false };
  double resolution_;
  double transform_max_wait_time_;
  int start_x_;   // start x (m)
  int start_y_;   // start y (m)
  int length_x_;  // length (m)
  int width_y_;   // width (m)
  int kernel_size_;

  bool debug_;
  double radius_;  // Radius to filter lidar points // TODO: Refactor to a new node
  double lidar_miss_cast_distance_;
  double filter_distance_;
  double blur_std_dev_;

  int resize_width_;
  int resize_height_;

  EmptyFilterOptions empty_filter_options_{};
  BehindFilterOptions behind_filter_options_{};

  radians filter_angle_;
  radians lidar_start_angle_;
  radians lidar_end_angle_;
  radians angular_resolution_;
  std::string lidar_topic_;
  std::string line_topic_;
  std::string camera_frame_;
  std::string projected_line_topic_;
  std::string camera_info_topic_;
  RobotState state_;                      // Odom -> Base_link
  RobotState odom_to_lidar_;              // Odom -> Lidar
  RobotState odom_to_camera_projection_;  // Odom -> Camera Projection

  image_geometry::PinholeCameraModel camera_model_;

  std::unique_ptr<Octomapper> octomapper_;
  pc_map_pair pc_map_pair_;      // Struct storing both the octomap for the lidar and the cv::Mat map
  pc_map_pair camera_map_pair_;  // Struct storing both the octomap for the camera projections and the cv::Mat map
};
#endif  // PROJECT_MAPPER_H
