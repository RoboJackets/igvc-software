/**
 * Class that interfaces with ROS publisher and subscribers, passing information to Mapper to be mapped.
 *
 * Author: Oswin So <oswinso@gatech.edu>
 * Date Created: March 24 2019
 */
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
  /**
   * Callback for pointcloud. Inserted into the map as a lidar scan.
   * @param[in] pc Lidar scan
   */
  void pcCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc);

  /**
   * Callback for the neural network segmented image. Projected, then inserted as points.
   * @param segmented the segmented image
   * @param camera which Mapper::Camera
   */
  void segmentedImageCallback(const sensor_msgs::ImageConstPtr& segmented, Camera camera);

  /**
   * Callback for the line projetced onto lidar point cloud. Directly inserted as points.
   * @param pc the pointcloud from the projection of the segmented image onto lidar data.
   * @param camera the camera from which this pointcloud comes from
   */
  void projectedLineCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc, Camera camera);

  /**
   * Callback for the cameraInfo used for projection, modified for a resized camera image.
   * @param camera_info CameraInfo used for projection.
   */
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info, Camera camera);

  /**
   * Publishes the given map at the given stamp
   * @param[in] stamp timestamp to be used for the publishing
   */
  void publish(uint64_t stamp);

  /**
   * Populates <code>igvc_msgs::map message</code> with information from <code>sensor_msgs::Image</code> and the
   * timestamp from <code>pcl_stamp</code>
   * @param[out] message message to be filled out
   * @param[in] image image containing map data to be put into <code>message</code>
   * @param[in] pcl_stamp time stamp from the pcl to be used for <code>message</code>
   */
  void setMessageMetadata(igvc_msgs::map& message, sensor_msgs::Image& image, uint64_t pcl_stamp);

  /**
   * Checks that there is an existing static transform from base_footprint to frame_id. If there isn't,
   * find the given transform using the timestamp provided.
   * @tparam[in] T timestamp type, either ros::Time or uint64
   * @param[in] frame_id frame_id to check for
   * @param[in] timestamp the time to check for
   * @param[in] topic the key to be used to store the static transform
   * @return true if a static transform was found, false otherwise.
   */
  template <class T>
  bool checkExistsStaticTransform(const std::string& frame_id, T timestamp, const std::string& topic);

  /**
   * Updates <code>RobotState state</code> with the latest tf transform using the timestamp of the message passed in
   * @param[in] msg <code>pcl::PointCloud</code> message with the timestamp used for looking up the tf transform
   */
  template <class T>
  bool getOdomTransform(const T stamp);

  /**
   * Publishes the passed in cv::Mat as a pointcloud on the given publisher
   * @param[in] pub publisher to use
   * @param[in] mat the map to use
   * @param[in] frame_id the frame to use
   * @param[in] stamp the stamp to use
   */
  void publishMapDebugPC(const ros::Publisher& pub, const cv::Mat& mat, const std::string& frame_id, uint64_t stamp);

  cv_bridge::CvImage img_bridge_;

  ros::Publisher map_pub_;                                  // Publishes map
  ros::Publisher debug_pcl_pub_;                            // Publishes blumap as individual PCL points
  std::map<std::string, tf::StampedTransform> transforms_;  // Map of static transforms
  std::unique_ptr<tf::TransformListener> tf_listener_;      // TF Listener

  std::unordered_map<Camera, ros::Subscriber> camera_infos_;
  std::unordered_map<Camera, ros::Subscriber> line_map_subs_;
  std::unordered_map<Camera, ros::Subscriber> projected_line_subs_;

  bool use_lines_{};
  double transform_max_wait_time_{};
  int start_x_{};   // start x (m)
  int start_y_{};   // start y (m)
  int length_x_{};  // length (m)
  int width_y_{};   // width (m)

  bool debug_pub_map_pcl{};

  bool enable_left_cam_;
  bool enable_center_cam_;
  bool enable_right_cam_;

  bool use_passed_in_pointcloud_;

  double resolution_{};  // Map Resolution

  int resize_width_{};
  int resize_height_{};

  std::string lidar_topic_;

  std::string line_topic_left_;
  std::string line_topic_center_;
  std::string line_topic_right_;

  std::string projected_line_topic_left_;
  std::string projected_line_topic_center_;
  std::string projected_line_topic_right_;

  std::string camera_info_topic_left_;
  std::string camera_info_topic_center_;
  std::string camera_info_topic_right_;

  std::string camera_frame_left_;
  std::string camera_frame_center_;
  std::string camera_frame_right_;
  RobotState state_;  // Odom -> Base_link

  std::unique_ptr<Mapper> mapper_;
};
#endif  // PROJECT_MAPPER_H
