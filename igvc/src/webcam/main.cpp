#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <camera_info_manager/camera_info_manager.h>
#include <ros/publisher.h>
#include <opencv2/video/video.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"

// ros::Publisher info_pub;
// camera_info_manager::CameraInfoManager* cameraManager;

using namespace cv;
cv_bridge::CvImagePtr cv_ptr;
image_transport::Publisher _new_img;

void callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_ptr = cv_bridge::toCvCopy(msg, "");
    Mat image = (cv_ptr->image).clone();

    // All of my code goes here
    Mat img_overhead = imread("overhead.jpg", CV_LOAD_IMAGE_GRAYSCALE );
    Mat img_normal = imread("normal.jpg", CV_LOAD_IMAGE_GRAYSCALE );
    

    // namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE );
    // imshow("calcHist Demo", histImage );

    cv_ptr->image = img_normal;
    _new_img.publish(cv_ptr->toImageMsg());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "webcam");
    ros::NodeHandle nh;

    // cameraManager = new camera_info_manager::CameraInfoManager(ros::NodeHandle("/usb_cam"), "/usb_cam", "file:///home/nareddyt/Desktop/cal.yml");
    // info_pub = nh.advertise<sensor_msgs::CameraInfo>("/usb_cam/camera_info", 1);

    ros::Subscriber sub = nh.subscribe("/usb_cam/image_raw", 1, callback);

    image_transport::ImageTransport _it(nh);
    _new_img = _it.advertise("/new_img", 1);

    ros::spin();
}