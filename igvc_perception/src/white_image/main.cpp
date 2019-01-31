#include <ros/publisher.h>
#include <ros/ros.h>
#include <igvc_utils/NodeUtils.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "white_image");

  ros::NodeHandle n;

  ros::Publisher _white_image_pub = n.advertise<sensor_msgs::Image>("test_image", 1);

  cv::Mat test_image = cv::Mat(1080, 1920, CV_8UC3, cv::Scalar(255, 255, 255));
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", test_image).toImageMsg();
  while (ros::ok())
  {
    _white_image_pub.publish(msg);
  }
}