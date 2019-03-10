#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <fstream>
#include <iostream>
#include <igvc_utils/NodeUtils.hpp>

ros::Publisher img_pub_lines;
ros::Publisher img_pub_barrels;
sensor_msgs::Image outmsg_lines;
sensor_msgs::Image outmsg_barrels;

/*
 * Responsible for detecting the lines (currently set to a purple color) and
 * illuminating them as white (and setting all other pixels to black).
 */
void handle_image_lines(const sensor_msgs::ImageConstPtr& msg) {

  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat frame;

  try {
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("CV-Bridge error: %s", e.what());
    return;
  }

  frame = cv_ptr->image;
  cv::Mat output(frame.rows, frame.cols, CV_8UC1, cv::Scalar::all(0));

  // Set color ranges (8 bit BGR) for line detection
  const int min_b = 120;
  const int max_b = 140;
  const int min_g = 0;
  const int max_g = 20;
  const int min_r = 70;
  const int max_r = 85;

  const int white_color = 255;
  const int black_color = 0;

  for(int rowCount=0; rowCount<frame.rows; ++rowCount) {
    for(int columnCount=0; columnCount<frame.cols; ++columnCount)  {

      cv::Vec3b color = frame.at<cv::Vec3b>(cv::Point(columnCount,rowCount));

      if (int(color.val[0]) > min_b && int(color.val[0]) < max_b && int(color.val[1]) >= min_g && int(color.val[1]) < max_g && int(color.val[2]) > min_r && int(color.val[2]) < max_r) {
        output.at<uchar>(cv::Point(columnCount, rowCount)) = white_color;
      } else {
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
void handle_image_barrels(const sensor_msgs::ImageConstPtr& msg) {

  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat frame;

  try {
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("CV-Bridge error: %s", e.what());
    return;
  }

  frame = cv_ptr->image;
  cv::Mat output(frame.rows, frame.cols, CV_8UC1, cv::Scalar::all(0));

  // Set color ranges (8 bit BGR) for barrel detection
  // Detects the black base of the barrel
  const int max_b_obstacle_black = 45;
  const int max_g_obstacle_black = 45;
  const int max_r_obstacle_black = 45;

  // Detects the orange parts of the barrel
  const int max_g_obstacle_orange = 160;
  const int min_r_obstacle_orange = 240;

  // Detects the white parts of the barrel
  const int min_b_obstacle_white = 230;
  const int min_g_obstacle_white = 230;
  const int min_r_obstacle_white = 230;

  const int white_color = 255;
  const int black_color = 0;

  for(int rowCount=0; rowCount<frame.rows; ++rowCount) {
    for(int columnCount=0; columnCount<frame.cols; ++columnCount)  {

      cv::Vec3b color = frame.at<cv::Vec3b>(cv::Point(columnCount,rowCount));

      if (int(color.val[0]) < max_b_obstacle_black && int(color.val[1]) < max_g_obstacle_black && int(color.val[2]) < max_r_obstacle_black) {
        output.at<uchar>(cv::Point(columnCount, rowCount)) = white_color;
      } else if (int(color.val[1]) < max_g_obstacle_orange && int(color.val[2]) > min_r_obstacle_orange) {
        output.at<uchar>(cv::Point(columnCount, rowCount)) = white_color;
      } else if (int(color.val[0]) > min_b_obstacle_white && int(color.val[1]) > min_g_obstacle_white && int(color.val[2]) > min_r_obstacle_white) {
        output.at<uchar>(cv::Point(columnCount, rowCount)) = white_color;
      } else {
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
int main(int argc, char** argv) {

  ros::init(argc, argv, "sim_color_detector");
  
  // LINES

  ros::NodeHandle nh_lines;
  ros::NodeHandle pNh_lines("~");

  std::string topic_name_lines;
  
  igvc::param(pNh_lines, "image_topic", topic_name_lines, std::string("/center_cam/image_raw"));

  img_pub_lines = nh_lines.advertise<sensor_msgs::Image>("/usb_cam_center/detected_lines", 1);

  ros::Subscriber img_sub_lines = nh_lines.subscribe(topic_name_lines, 1, handle_image_lines);

  // BARRELS

  ros::NodeHandle nh_barrels;
  ros::NodeHandle pNh_barrels("~");

  std::string topic_name_barrels;
  
  igvc::param(pNh_barrels, "image_topic", topic_name_barrels, std::string("/center_cam/image_raw"));

  img_pub_barrels = nh_barrels.advertise<sensor_msgs::Image>("/usb_cam_center/detected_barrels", 1);

  ros::Subscriber img_sub_barrels = nh_barrels.subscribe(topic_name_barrels, 1, handle_image_barrels);

  ros::NodeHandle nh;

  // Will call publish to update the published topics -> Set the publishing rate here
  ros::Timer timer = nh.createTimer(ros::Duration(1.0/30), publish);

  ros::spin();

  return 0;
}
