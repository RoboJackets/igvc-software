#include "linedetector.h"
#include <igvc/CVUtils.hpp>

cv_bridge::CvImagePtr cv_ptr;
typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

void LineDetector::info_img_callback(const sensor_msgs::ImageConstPtr& msg,
                                     const sensor_msgs::CameraInfoConstPtr& cam_info)
{
  cv_bridge::CvImagePtr cv_copy;
  cv_copy = cv_bridge::toCvCopy(msg, "");
  cv::Mat img = cv_copy->image;

  img = ResizeCameraImage(img, 640, 360);
  sensor_msgs::CameraInfo cam_info_rsz = ResizeCameraInfo(cam_info, 640, 360);

  cam.fromCameraInfo(cam_info_rsz);
  img_callback(img, msg);
}

void LineDetector::img_callback(const cv::Mat msg_img, const sensor_msgs::ImageConstPtr& origMsg)
{
  src_img = msg_img;

  cv::Mat colors[3];
  split(src_img, colors);

  src_img = colors[2];

  // Convert image to grayscale
  // cv::cvtColor(src_img, src_img, CV_BGR2GRAY);

  // Crops the image (removes sky)
  int topCrop = 2 * src_img.rows / 3;
  cv::Rect roiNoSky(0, topCrop, src_img.cols, src_img.rows - topCrop);
  src_img = src_img(roiNoSky);

  // Create blank canvas
  cv::Mat working2 = cv::Mat::zeros(src_img.size(), src_img.type());
  fin_img = cv::Mat::zeros(src_img.size(), src_img.type());

  // Gaussian Blur
  cv::GaussianBlur(src_img, working, cv::Size(3, 3), 2.0);

  // Detect edges
  cv::Canny(working, working, cannyThresh, cannyThresh * ratio, 3);

  // Erosion and dilation
  int kernel_size = 3;
  cv::Mat erosion_kernal = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(kernel_size, kernel_size));
  cv::dilate(working, working, erosion_kernal);
  cv::dilate(working, working, erosion_kernal);
  cv::erode(working, working, erosion_kernal);
  cv::erode(working, working, erosion_kernal);

  // Find lines
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(working, lines, 1.0, CV_PI / 180, houghThreshold, houghMinLineLength, houghMaxLineGap);
  for (size_t i = 0; i < lines.size(); ++i)
  {
    cv::LineIterator it(working2, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), 8);
    for (int j = 0; j < it.count; ++j, ++it)
    {
      if (j % outputLineSpacing == 0)
        working2.at<uchar>(it.pos()) = 255;
    }
  }

  // Erosion and dilation
  // cv::dilate(working2, working2, erosion_kernal);
  // cv::dilate(working2, working2, erosion_kernal);
  // cv::erode(working2, working2, erosion_kernal);
  // cv::erode(working2, working2, erosion_kernal);
  // cv::erode(working2, working2, erosion_kernal);
  // cv::erode(working2, working2, erosion_kernal);

  // Detect edges
  // cv::Canny(working2, working2, cannyThresh, cannyThresh*ratio, 3);

  // Find lines
  std::vector<cv::Vec4i> lines2;
  cv::HoughLinesP(working2, lines2, 1.0, CV_PI / 180, houghThreshold, houghMinLineLength * 2, houghMaxLineGap);
  for (size_t i = 0; i < lines2.size(); ++i)
  {
    cv::LineIterator it(fin_img, cv::Point(lines2[i][0], lines2[i][1]), cv::Point(lines2[i][2], lines2[i][3]), 8);
    for (int j = 0; j < it.count; ++j, ++it)
    {
      if (j % outputLineSpacing == 0)
        fin_img.at<uchar>(it.pos()) = 255;
    }
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
  cv_bridge::CvImagePtr newPtr = cv_bridge::toCvCopy(origMsg, "");
  newPtr->image = fin_img;
  _filt_img.publish(newPtr->toImageMsg());
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
  handle.getParam(ros::this_node::getName() + "/config/line/outputLineSpacing", outputLineSpacing);
}
