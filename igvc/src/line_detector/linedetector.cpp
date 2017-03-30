#include "linedetector.h"
#include <igvc/CVUtils.hpp>

cv_bridge::CvImagePtr cv_ptr;
typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

void LineDetector::info_img_callback(const sensor_msgs::ImageConstPtr& msg,
                                     const sensor_msgs::CameraInfoConstPtr& cam_info)
{
  cam.fromCameraInfo(cam_info);
  img_callback(msg);
}

void LineDetector::img_callback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_ptr = cv_bridge::toCvCopy(msg, "");
  src_img = cv_ptr->image;

  // Convert image to grayscale
  cv::cvtColor(src_img, src_img, CV_BGR2GRAY);

  // Crops the image (removes sky)
  int topCrop = 2 * src_img.rows / 3;
  cv::Rect roiNoSky(0, topCrop, src_img.cols, src_img.rows - topCrop);
  src_img = src_img(roiNoSky);

  // Create blank canvas
  fin_img = cv::Mat::zeros(src_img.size(), src_img.type());

  // Gaussian Blur
  cv::GaussianBlur(src_img, working, cv::Size(3, 3), 2.0);

  // Detect edges
  cv::Canny(working, working, cannyThresh, cannyThresh*ratio, 3);

  // Erosion and dilation
  int kernel_size = 3;
  cv::Mat erosion_kernal = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(kernel_size, kernel_size));
  cv::dilate(working, working, erosion_kernal);
  cv::dilate(working, working, erosion_kernal);
  cv::dilate(working, working, erosion_kernal);
  cv::erode(working, working, erosion_kernal);
  cv::erode(working, working, erosion_kernal);
  cv::erode(working, working, erosion_kernal);

  // Find lines
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(working, lines, 1.0, CV_PI / 180, houghThreshold, houghMinLineLength, houghMaxLineGap);
  for (size_t i = 0; i < lines.size(); ++i)
  {
    cv::line(fin_img, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]),
             cv::Scalar(255, 255, 255), 1, 4);
  }

  // Re-fill sky area of image with black
  cv::Mat black = cv::Mat::zeros(cv::Size(src_img.cols, topCrop), src_img.type());
  cv::vconcat(black, fin_img, fin_img);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

  cloud = toPointCloud(tf_listener, MatToContours(fin_img), cam, topic);

  // Limit how far the points are plotted in the cloud
  capPointCloud(cloud, maxDistance);

  _line_cloud.publish(cloud);

  cv::cvtColor(fin_img, fin_img, CV_GRAY2BGR);

  cv_ptr->image = fin_img;
  _filt_img.publish(cv_ptr->toImageMsg());
}

LineDetector::LineDetector(ros::NodeHandle& handle, const std::string& topic)
  : _it(handle), topic(topic), tf_listener(handle)
{
  _src_img_info = _it.subscribeCamera(topic + "/image_raw", 1, &LineDetector::info_img_callback, this);
  _filt_img = _it.advertise(topic + "/filt_img", 1);
  _line_cloud = handle.advertise<PCLCloud>(topic + "/line_cloud", 100);

  handle.getParam(ros::this_node::getName() + "/config/line/cannyThresh", cannyThresh);
  handle.getParam(ros::this_node::getName() + "/config/line/ratio", ratio);
  handle.getParam(ros::this_node::getName() + "/config/line/houghThreshold", houghThreshold);
  handle.getParam(ros::this_node::getName() + "/config/line/houghMinLineLength", houghMinLineLength);
  handle.getParam(ros::this_node::getName() + "/config/line/houghMaxLineGap", houghMaxLineGap);
  handle.getParam(ros::this_node::getName() + "/config/line/maxDistance", maxDistance);
}
