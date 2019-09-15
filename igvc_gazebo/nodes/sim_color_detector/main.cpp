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

#include <parameter_assertions/assertions.h>
#include <map>
#include <vector>

using Pixel = cv::Point3_<uint8_t>;

// map of camera name to line and barrel publishers
std::map<std::string, std::vector<ros::Publisher>> g_pubs;

/*
 * Holds all the ranges for a color being detected in the simulator;
 * done for improved readability
 */
struct Color
{
  Color(int min_b, int max_b, int min_g, int max_g, int min_r, int max_r)
  {
    _min_b = min_b;
    _max_b = max_b;
    _min_g = min_g;
    _max_g = max_g;
    _min_r = min_r;
    _max_r = max_r;
  }

  int _min_b;
  int _max_b;
  int _min_g;
  int _max_g;
  int _min_r;
  int _max_r;
};

const Color PURPLE = Color(120, 140, 0, 20, 70, 85);
const Color ORANGE = Color(0, 255, 0, 160, 240, 255);
const Color WHITE = Color(230, 255, 230, 255, 230, 255);
const Color BLACK = Color(0, 45, 0, 45, 0, 45);

/*
 * For readability and conciseness, compares the color of the pixel with the desired color.
 * Returns true if the pixel_color is in the bounds for B, G, and R; returns false otherwise
 */
bool color_check(const Pixel* pixel, Color desired_color)
{
  int b_value = static_cast<int>(pixel->x);
  int g_value = static_cast<int>(pixel->y);
  int r_value = static_cast<int>(pixel->z);

  if (b_value >= desired_color._min_b && b_value <= desired_color._max_b)
  {
    if (g_value >= desired_color._min_g && g_value <= desired_color._max_g)
    {
      if (r_value >= desired_color._min_r && r_value <= desired_color._max_r)
      {
        return true;
      }
    }
  }

  return false;
}

/**
Recieves an input image and publishes two segmented images: one for lines and
another for barrels
*/
void handleImage(const sensor_msgs::ImageConstPtr& msg, std::string camera_name)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat frame;  // Input image

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    frame = cv_ptr->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("CV-Bridge error: %s", e.what());
    return;
  }

  cv::Mat output_lines(frame.rows, frame.cols, CV_8UC1, cv::Scalar::all(0));    // Ouput image lines (B&W)
  cv::Mat output_barrels(frame.rows, frame.cols, CV_8UC1, cv::Scalar::all(0));  // Ouput image barrels (B&W)

  const int white_color = 255;
  const int black_color = 0;

  // segment lines and barrels
  for (int r = 0; r < frame.rows; ++r)
  {
    int c = 0;
    Pixel* ptr = frame.ptr<Pixel>(r, 0);
    const Pixel* ptr_end = ptr + frame.cols;

    for (; ptr != ptr_end; ++ptr, c++)
    {
      // segment lines
      if (color_check(ptr, PURPLE))
        output_lines.at<uchar>(cv::Point(c, r)) = white_color;
      else
        output_lines.at<uchar>(cv::Point(c, r)) = black_color;

      // segment barrels
      if (color_check(ptr, BLACK) || color_check(ptr, ORANGE) || color_check(ptr, WHITE))
        output_barrels.at<uchar>(cv::Point(c, r)) = white_color;
      else
        output_barrels.at<uchar>(cv::Point(c, r)) = black_color;
    }
  }

  sensor_msgs::Image outmsg;
  outmsg.header = msg->header;
  cv_ptr->encoding = "mono8";

  // publish line segmentation
  cv_ptr->image = output_lines;
  cv_ptr->toImageMsg(outmsg);
  g_pubs.at(camera_name).at(0).publish(outmsg);

  // publish barrel segmentation
  cv_ptr->image = output_barrels;
  cv_ptr->toImageMsg(outmsg);
  g_pubs.at(camera_name).at(1).publish(outmsg);
}

/*
 * Responsible for setting up the publishers and triggering the functions to
 * update the detected black-and-white images
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "sim_color_detector");
  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  // cameras to obtain images from
  std::vector<std::string> camera_names;
  assertions::getParam(pNh, "camera_names", camera_names);

  // output topics
  std::string line_topic;
  assertions::getParam(pNh, "line_topic", line_topic);
  std::string barrel_topic;
  assertions::getParam(pNh, "barrel_topic", barrel_topic);

  // insert subscribers and publishers
  std::vector<ros::Subscriber> subs;
  for (std::string camera_name : camera_names)
  {
    // subscribe to raw camera image
    ros::Subscriber cam_sub =
        nh.subscribe<sensor_msgs::Image>(camera_name + "/image_raw", 1, boost::bind(handleImage, _1, camera_name));
    subs.push_back(cam_sub);

    // publish line and barrel segmentation
    ros::Publisher line_pub = nh.advertise<sensor_msgs::Image>(camera_name + line_topic, 1);
    ros::Publisher barrel_pub = nh.advertise<sensor_msgs::Image>(camera_name + barrel_topic, 1);
    std::vector<ros::Publisher> camera_pubs = { line_pub, barrel_pub };

    g_pubs.insert(std::make_pair(camera_name, camera_pubs));
  }

  ros::spin();
}
