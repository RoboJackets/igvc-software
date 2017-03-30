#ifndef LINEDETECTOR_H
#define LINEDETECTOR_H

#include "igvc/CVInclude.h"

#define KERNAL_COUNT 8

class LineDetector
{
public:
  LineDetector(ros::NodeHandle& handle, const std::string& topic);

private:
  cv::Mat src_img, working, fin_img;

  void info_img_callback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info);
  void img_callback(const sensor_msgs::ImageConstPtr& msg);

  // ROS COMMUNICATION
  image_transport::ImageTransport _it;
  std::string topic;
  image_transport::Publisher _filt_img;
  image_transport::Subscriber _src_img;
  image_transport::CameraSubscriber _src_img_info;
  ros::Publisher _line_cloud;
  tf::TransformListener tf_listener;
  image_geometry::PinholeCameraModel cam;

  // Tuning parameters loaded from YAML file (file specified in launch file)
  int cannyThresh;
  int ratio;
  int houghThreshold;
  int houghMinLineLength;
  int houghMaxLineGap;
  int maxDistance;
};

#endif  // LINEDETECTOR_H
