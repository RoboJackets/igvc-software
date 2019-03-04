#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <fstream>
#include <iostream>
#include <igvc_utils/NodeUtils.hpp>

ros::Publisher img_pub;

void handle_image(const sensor_msgs::ImageConstPtr& msg) {

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

  const int min_b_line = 120;
  const int max_b_line = 140;
  const int min_g_line = 0;
  const int max_g_line = 20;
  const int min_r_line = 70;
  const int max_r_line = 85;

  const int max_b_obstacle_black = 45;
  const int max_g_obstacle_black = 45;
  const int max_r_obstacle_black = 45;

  const int max_g_obstacle_orange = 160;
  const int min_r_obstacle_orange = 240;

  const int min_b_obstacle_white = 230;
  const int min_g_obstacle_white = 230;
  const int min_r_obstacle_white = 230;

  const int white_color = 255;
  const int black_color = 0;

  for(int rowCount=0; rowCount<frame.rows; ++rowCount) {
    for(int columnCount=0; columnCount<frame.cols; ++columnCount)  {

      cv::Vec3b color = frame.at<cv::Vec3b>(cv::Point(columnCount,rowCount));

      if (int(color.val[0]) > min_b_line && int(color.val[0]) < max_b_line && int(color.val[1]) >= min_g_line && int(color.val[1]) < max_g_line && int(color.val[2]) > min_r_line && int(color.val[2]) < max_r_line) {
        output.at<uchar>(cv::Point(columnCount, rowCount)) = white_color;
      } else if (int(color.val[0]) < max_b_obstacle_black && int(color.val[1]) < max_g_obstacle_black && int(color.val[2]) < max_r_obstacle_black) {
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
  img_pub.publish(outmsg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "sim_color_detector");

  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  std::string topic_name;
  
  //pNh.param("image_topic", topic_name, std::string("/center_cam/image_raw"));

  igvc::param(pNh, "image_topic", topic_name, std::string("/center_cam/image_raw"));

  img_pub = nh.advertise<sensor_msgs::Image>("/usb_cam_center/detected", 1);

  ros::Subscriber img_sub = nh.subscribe(topic_name, 1, handle_image);

  ros::spin();

  return 0;
}
