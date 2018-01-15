#ifndef FLAGDETECTOR_H
#define FLAGDETECTOR_H
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <math.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/video.hpp>
#include <vector>

using namespace std;
using namespace cv;
using namespace pcl;

class FlagDetector
{
public:
  FlagDetector(ros::NodeHandle& handle);
  PointCloud<pcl::PointXYZ>::Ptr cloud;

private:
  void camera_info_callback(const sensor_msgs::CameraInfoPtr& msg);
  void img_callback(const sensor_msgs::ImageConstPtr& msg);

  PointCloud<PointXYZ>::Ptr toPointCloud(vector<vector<Point>>& contours, int height, int width);

  /**
   * @brief gaussian_size The size of the Gaussian blur. The bigger the greater the blur
   * @note Must be odd!
   */
  const int gaussian_size;

  /**
   * @brief src contains the original, unprocessed image
   */
  // cv::Mat src;
  /**
   * @brief dst contains the new, processed image that isolates the lines
   */
  // cv::Mat *dst;

  // ROS COMMUNICATION
  image_transport::ImageTransport _it;
  image_transport::Publisher _flag_filt_img;
  image_transport::Publisher _flag_thres;
  image_transport::Subscriber _src_img;
  ros::Subscriber _camera_info;
  ros::Publisher _flag_cloud;
  tf::TransformListener tf_listener;

  Mat src;
  Mat src_gray;
};
#endif  // POTHOLEDETECTOR_H
