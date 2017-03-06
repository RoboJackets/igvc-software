#ifndef LINEDETECTOR_H
#define LINEDETECTOR_H

#include "igvc/CVInclude.h"

#define KERNAL_COUNT 8

class LineDetector
{
public:
  LineDetector(ros::NodeHandle& handle, const std::string& topic);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

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
  bool hasInfo = true;

  // Tuning parameters loaded from YAML file (file specified in launch file)
  int cannyThresh1;
  int cannyThresh2;
  int houghThreshold;
  int houghMinLineLength;
  int houghMaxLineGap;
};

#endif  // LINEDETECTOR_H
