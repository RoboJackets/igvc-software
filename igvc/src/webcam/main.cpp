#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <camera_info_manager/camera_info_manager.h>
#include <ros/publisher.h>
#include <opencv2/video/video.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// ros::Publisher info_pub;
// camera_info_manager::CameraInfoManager* cameraManager;

using namespace cv;
cv_bridge::CvImagePtr cv_ptr;
image_transport::Publisher _new_img;

void callback(const sensor_msgs::ImageConstPtr& msg) {
    
    // sensor_msgs::CameraInfo cl = cameraManager->getCameraInfo();
    // cl.header.frame_id = "/usb_cam";
    // cl.header.stamp = msg->header.stamp;
    // cameraManager->setCameraInfo(cl);

    // info_pub.publish(cameraManager->getCameraInfo());

    cv_ptr = cv_bridge::toCvCopy(msg, "");
    Mat image = (cv_ptr->image).clone();

    // To access an element in the image (aka a pixel)
    // image.at<double>(x, y);

    // All of my code goes here
    Mat dst;

    vector<Mat> bgr_planes;
    split(image, bgr_planes);

    int histSize = 256;
    float range[] = {0, 256};
    const float* histRange = {range};
    bool uniform = true;
    bool accumulate = false;

    Mat b_hist, g_hist, r_hist;

    calcHist( &bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
    calcHist( &bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
    calcHist( &bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );

    int hist_w = 512; int hist_h = 400;
    int bin_w = cvRound( (double) hist_w/histSize );

    Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

    normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

    for( int i = 1; i < histSize; i++ ) {
        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
                         Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
                         Scalar( 255, 0, 0), 2, 8, 0  );
        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
                         Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
                         Scalar( 0, 255, 0), 2, 8, 0  );
        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ) ,
                         Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
                         Scalar( 0, 0, 255), 2, 8, 0  );
    }

    namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE );
    imshow("calcHist Demo", histImage );

    cv_ptr->image = histImage;
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