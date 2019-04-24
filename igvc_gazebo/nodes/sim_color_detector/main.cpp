/*
 * Node responsible for (color) detecting the lines and barrels in the Gazebo
 * simulator. Receives the raw image and publishes two different B&W images
 * (one for line detection, one for barrel detection) that each highlight
 * their respective features in white. Integrated with a ros::Timer to publish at
 * a specified rate.
 */

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <fstream>
#include <igvc_utils/NodeUtils.hpp>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

ros::Publisher img_pub_lines;
ros::Publisher img_pub_barrels;
sensor_msgs::Image outmsg_lines;
sensor_msgs::Image outmsg_barrels;

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
bool color_check(cv::Vec3b pixel_color, Color desired_color)
{
  int b_value = int(pixel_color.val[0]);
  int g_value = int(pixel_color.val[1]);
  int r_value = int(pixel_color.val[2]);

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

/*
 * Responsible for detecting the lines (currently set to a purple color) and
 * illuminating them as white (and setting all other pixels to black).
 */
void handle_image_lines(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat frame;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("CV-Bridge error: %s", e.what());
    return;
  }

  frame = cv_ptr->image;                                                // Input image
  cv::Mat output(frame.rows, frame.cols, CV_8UC1, cv::Scalar::all(0));  // Ouput image (B&W)

  const int white_color = 255;
  const int black_color = 0;

  for (int rowCount = 0; rowCount < frame.rows; ++rowCount)
  {
    for (int columnCount = 0; columnCount < frame.cols; ++columnCount)
    {
      cv::Vec3b color = frame.at<cv::Vec3b>(cv::Point(columnCount, rowCount));

      if (color_check(color, PURPLE))
      {
        output.at<uchar>(cv::Point(columnCount, rowCount)) = white_color;
      }
      else
      {
        output.at<uchar>(cv::Point(columnCount, rowCount)) = black_color;
      }
    }
  }

  sensor_msgs::Image outmsg;
  outmsg.header = msg->header;
  cv_ptr->image = output;
  cv_ptr->encoding = "mono8";
  cv_ptr->toImageMsg(outmsg);
  outmsg_lines = outmsg;
}

/*
 * Responsible for detecting the barrels (white and orange components) and
 * illuminating them as white (and setting all other pixels to black).
 */
void handle_image_barrels(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat frame;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("CV-Bridge error: %s", e.what());
    return;
  }

  frame = cv_ptr->image;                                                // Input image
  cv::Mat output(frame.rows, frame.cols, CV_8UC1, cv::Scalar::all(0));  // Ouput image (B&W)

  const int white_color = 255;
  const int black_color = 0;

  for (int rowCount = 0; rowCount < frame.rows; ++rowCount)
  {
    for (int columnCount = 0; columnCount < frame.cols; ++columnCount)
    {
      cv::Vec3b color = frame.at<cv::Vec3b>(cv::Point(columnCount, rowCount));

      if (color_check(color, BLACK))
      {
        output.at<uchar>(cv::Point(columnCount, rowCount)) = white_color;
      }
      else if (color_check(color, ORANGE))
      {
        output.at<uchar>(cv::Point(columnCount, rowCount)) = white_color;
      }
      else if (color_check(color, WHITE))
      {
        output.at<uchar>(cv::Point(columnCount, rowCount)) = white_color;
      }
      else
      {
        output.at<uchar>(cv::Point(columnCount, rowCount)) = black_color;
      }
    }
  }

  sensor_msgs::Image outmsg;
  outmsg.header = msg->header;
  cv_ptr->image = output;
  cv_ptr->encoding = "mono8";
  cv_ptr->toImageMsg(outmsg);
  outmsg_barrels = outmsg;
}

/*
 * Takes the most recent messages from the handle_image_lines and
 * handle_image_barrels functions and publishes them once triggered
 * by a a ros::Timer
 */
void publish(const ros::TimerEvent&)
{
  img_pub_barrels.publish(outmsg_barrels);
  img_pub_lines.publish(outmsg_lines);
}

/*
 * Responsible for setting up the publishers and triggering the functions to
 * update the detected black-and-white images
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "sim_color_detector");

  // LINES

  ros::NodeHandle nh_lines;
  ros::NodeHandle pNh_lines("~");

  std::string topic_name_lines;

  igvc::param(pNh_lines, "image_topic", topic_name_lines, std::string("/center_cam/image_raw"));

  std::string name_for_topic_lines = topic_name_lines + "/detected_lines";

  img_pub_lines = nh_lines.advertise<sensor_msgs::Image>(name_for_topic_lines, 1);

  ros::Subscriber img_sub_lines = nh_lines.subscribe(topic_name_lines, 1, handle_image_lines);

  // BARRELS

  ros::NodeHandle nh_barrels;
  ros::NodeHandle pNh_barrels("~");

  std::string topic_name_barrels;

  igvc::param(pNh_barrels, "image_topic", topic_name_barrels, std::string("/center_cam/image_raw"));

  std::string name_for_topic_barrels = topic_name_barrels + "/detected_barrels";

  img_pub_barrels = nh_barrels.advertise<sensor_msgs::Image>(name_for_topic_barrels, 1);

  ros::Subscriber img_sub_barrels = nh_barrels.subscribe(topic_name_barrels, 1, handle_image_barrels);

  ros::NodeHandle nh;

  // Will call publish to update the published topics -> Set the publishing rate here

  const int rate = atoi(argv[1]);  // This is the rate for how often it should publish as an argument from command line

  ros::Timer timer = nh.createTimer(ros::Duration(1.0 / rate), publish);

  ros::spin();

  return 0;
}
