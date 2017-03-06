#ifndef POTHOLEDETECTOR_H
#define POTHOLEDETECTOR_H

#include "igvc/CVInclude.h"

#include <vector>

class PotholeDetector
{
public:
  PotholeDetector(ros::NodeHandle& handle, const std::string& topic);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

private:
  void img_callback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info);

  /**
   * @brief gaussian_size The size of the Gaussian blur. The bigger the greater the blur
   * @note Must be odd!
   */
  const int gaussian_size;

  // ROS COMMUNICATION
  image_transport::ImageTransport _it;
  image_transport::Publisher _pothole_filt_img;
  image_transport::Publisher _pothole_thres;
  image_transport::CameraSubscriber _src_img;
  ros::Subscriber _camera_info;
  ros::Publisher _pothole_cloud;
  tf::TransformListener tf_listener;
  std::string topic;

  cv::Mat src;
  cv::Mat src_gray;

  // Tuning parameters
  int maxRadius;
  int minRadius;
  int whiteSampleRadius;
  int contourSizeThreshold;
  int blueAdaptiveThreshold;
  int greenAdaptiveThreshold;
  int redAdaptiveThreshold;
};
#endif  // POTHOLEDETECTOR_H
