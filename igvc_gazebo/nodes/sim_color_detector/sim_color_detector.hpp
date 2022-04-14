/*
 * Node responsible for (color) detecting the lines and barrels in the Gazebo
 * simulator. Receives the raw image and publishes two different B&W images
 * (one for line detection, one for barrel detection) that each highlight
 * their respective features in white.
 */

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <fstream>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

#include <image_transport/image_transport.h>
#include <parameter_assertions/assertions.h>
#include <map>
#include <vector>

#include <parameter_assertions/assertions.h>

class SimColorDetector
{
  struct LineBarrelPair
  {
    image_transport::CameraPublisher camera;
    ros::Publisher barrel;
  };

public:
  SimColorDetector();

private:
  ros::NodeHandle nh;
  ros::NodeHandle pNh;

  std::vector<std::string> camera_names;
  std::vector<std::string> semantic_prefixes;
  std::vector<std::string> semantic_suffixes;
  std::vector<image_transport::CameraSubscriber> subs;

  // Output topics
  std::string line_topic;
  std::string barrel_topic;

  std::string image_base_topic;

  // Map of camera name to line and barrel publishers
  std::map<std::string, LineBarrelPair> g_pubs;

  // Output size of the image
  cv::Size g_output_size;

  cv::Scalar g_lower_lines;
  cv::Scalar g_upper_lines;

  sensor_msgs::CameraInfo scaleCameraInfo(const sensor_msgs::CameraInfo& camera_info, int width, int height);

  // Recieves an input image and publishes two segmented images: one for lines and another for barrels
  void handleImage(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& camera_info,
                   std::string camera_name);
};
