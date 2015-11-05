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
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/nonfree/features2d.hpp"

// ros::Publisher info_pub;
// camera_info_manager::CameraInfoManager* cameraManager;

using namespace cv;
cv_bridge::CvImagePtr cv_ptr;
image_transport::Publisher _new_img;

void callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_ptr = cv_bridge::toCvCopy(msg, "");
    Mat image = (cv_ptr->image).clone();

    // All of our code goes here
    Mat img_overhead = imread("/home/nareddyt/Desktop/IGVC/src/igvc-software/igvc/src/webcam/overhead.jpg", CV_LOAD_IMAGE_COLOR);
    Mat img_normal = imread("/home/nareddyt/Desktop/IGVC/src/igvc-software/igvc/src/webcam/normal.jpg", CV_LOAD_IMAGE_COLOR);

    //-- Step 1: Detect the keypoints using SURF Detector
      int minHessian = 400;

      SurfFeatureDetector detector(minHessian);

      std::vector<KeyPoint> keypoints_overhead, keypoints_normal;

      detector.detect( img_overhead, keypoints_overhead );
      detector.detect( img_normal, keypoints_normal );

      //-- Step 2: Calculate descriptors (feature vectors)
      SurfDescriptorExtractor extractor;

      Mat descriptors_overhead, descriptors_normal;

      extractor.compute( img_overhead, keypoints_overhead, descriptors_overhead );
      extractor.compute( img_normal, keypoints_normal, descriptors_normal );

      //-- Step 3: Matching descriptor vectors using FLANN matcher
      FlannBasedMatcher matcher;
      std::vector< DMatch > matches;
      matcher.match( descriptors_overhead, descriptors_normal, matches );

      double max_dist = 0; double min_dist = 100;

      //-- Quick calculation of max and min distances between keypoints
      for( int i = 0; i < descriptors_overhead.rows; i++ )
      { double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
      }

      //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
      std::vector< DMatch > good_matches;

      for( int i = 0; i < descriptors_overhead.rows; i++ )
      {
        if( matches[i].distance < 3*min_dist ) {
            good_matches.push_back( matches[i]); }
      }

      Mat img_matches;
    drawMatches( img_overhead, keypoints_overhead, img_normal, keypoints_normal,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

      //-- Localize the object
      std::vector<Point2f> obj;
      std::vector<Point2f> scene;

      for( int i = 0; i < good_matches.size(); i++ )
      {
        //-- Get the keypoints from the good matches
        obj.push_back( keypoints_overhead[ good_matches[i].queryIdx ].pt );
        scene.push_back( keypoints_normal[ good_matches[i].trainIdx ].pt );
      }

      Mat H = findHomography( obj, scene, CV_RANSAC, 3);

      //-- Get the corners from the image_1 ( the object to be "detected" )
  std::vector<Point2f> obj_corners(4);
  obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_overhead.cols, 0 );
  obj_corners[2] = cvPoint( img_overhead.cols, img_overhead.rows ); obj_corners[3] = cvPoint( 0, img_overhead.rows );
  std::vector<Point2f> scene_corners(4);

  perspectiveTransform( obj_corners, scene_corners, H);

  //-- Draw lines between the corners (the mapped object in the scene - image_2 )
  line( img_matches, scene_corners[0] + Point2f( img_overhead.cols, 0), scene_corners[1] + Point2f( img_overhead.cols, 0), Scalar(0, 255, 0), 4 );
  line( img_matches, scene_corners[1] + Point2f( img_overhead.cols, 0), scene_corners[2] + Point2f( img_overhead.cols, 0), Scalar( 0, 255, 0), 4 );
  line( img_matches, scene_corners[2] + Point2f( img_overhead.cols, 0), scene_corners[3] + Point2f( img_overhead.cols, 0), Scalar( 0, 255, 0), 4 );
  line( img_matches, scene_corners[3] + Point2f( img_overhead.cols, 0), scene_corners[0] + Point2f( img_overhead.cols, 0), Scalar( 0, 255, 0), 4 );

      std::cerr << "H = " << std::endl << H << std::endl << std::endl;

      cv::imwrite("/home/nareddyt/Desktop/IGVC/src/igvc-software/igvc/src/webcam/matches.jpg", img_matches);

      // Mat img_transformed;
      // warpPerspective(img_normal, img_transformed, H, img_transformed.size());

      // cv::imwrite("/home/nareddyt/Desktop/IGVC/src/igvc-software/igvc/src/webcam/transformed.jpg", img_transformed);

     //cv_ptr->image = img_transformed;
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