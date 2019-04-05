#ifndef PROJECT_MAPPER_H
#define PROJECT_MAPPER_H

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <ros/publisher.h>
#include "mapper.h"

class ROSMapper
{
  using radians = double;

public:
  ROSMapper();

private:
  // Callbacks
  void pcCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc);

  void segmentedImageCallback(const sensor_msgs::ImageConstPtr& segmented);

  void projctedLineCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc);

  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info);

  void publish(uint64_t stamp);

  void setMessageMetadata(igvc_msgs::map& message, sensor_msgs::Image& image, uint64_t pcl_stamp);

  /**
   * Checks that there is an existing static transform from base_footprint to frame_id. If there isn't,
   * find the given transform using the timestamp provided.
   * @tparam T
   * @param frame_id
   * @param timestamp
   * @param topic
   * @return
   */
  template <class T>
  bool checkExistsStaticTransform(const std::string& frame_id, T timestamp, const std::string& topic);

  template <class T>
  bool getOdomTransform(const T stamp);

  /**
   * Publishes the passed in cv::Mat as a pointcloud on the given publisher
   * @param pub
   * @param mat
   * @param frame_id
   * @param stamp
   */
  void publishAsPCL(const ros::Publisher& pub, const cv::Mat& mat, const std::string& frame_id, uint64_t stamp);

  cv_bridge::CvImage img_bridge_;

  ros::Publisher map_pub_;                                  // Publishes map
  ros::Publisher debug_pcl_pub_;                            // Publishes blumap as individual PCL points
  ros::Publisher debug_blurred_pc_;                         // Publishes blurred map as individual PCL points
  std::map<std::string, tf::StampedTransform> transforms_;  // Map of static transforms
  std::unique_ptr<tf::TransformListener> tf_listener_;      // TF Listener

  bool use_lines_;
  bool camera_model_initialized_{ false };
  double transform_max_wait_time_;
  int start_x_;   // start x (m)
  int start_y_;   // start y (m)
  int length_x_;  // length (m)
  int width_y_;   // width (m)

  bool debug_;

  double resolution_;  // Map Resolution

  int resize_width_;
  int resize_height_;

  std::string lidar_topic_;
  std::string line_topic_;
  std::string projected_line_topic_;
  std::string camera_info_topic_;
  RobotState state_;  // Odom -> Base_link

  std::unique_ptr<Mapper> mapper_;
};
#endif  // PROJECT_MAPPER_H
