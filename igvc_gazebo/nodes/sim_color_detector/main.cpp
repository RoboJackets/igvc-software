#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <fstream>
#include <iostream>

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

    for(int rowCount=0; rowCount<frame.rows; ++rowCount) {
        for(int columnCount=0; columnCount<frame.cols; ++columnCount)  {

	    	cv::Vec3b color = frame.at<cv::Vec3b>(cv::Point(columnCount,rowCount));

	    	if (int(color.val[0])>120 && int(color.val[0]) < 140 && int(color.val[1]) < 20 && int(color.val[2]) > 70 && int(color.val[2]) < 85) {
				output.at<uchar>(cv::Point(columnCount, rowCount)) = 255;
			} else {
				output.at<uchar>(cv::Point(columnCount, rowCount)) = 0;
			}
		
   		}
	}

	sensor_msgs::Image outmsg;
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
  pNh.param("image_topic", topic_name, std::string("/center_cam/image_raw"));

  img_pub = nh.advertise<sensor_msgs::Image>("/usb_cam_center/detected", 1);

  ros::Subscriber img_sub = nh.subscribe(topic_name, 1, handle_image);

  ros::spin();

  return 0;

}
