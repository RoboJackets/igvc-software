#include "potholedetector.h"
#include <opencv2/video/video.hpp>
#include <sensor_msgs/image_encodings.h>
#include <pcl_ros/point_cloud.h>
#include <camera_info_manager/camera_info_manager.h>
#include <math.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;
using namespace cv;
using namespace pcl;

cv_bridge::CvImagePtr cv_ptr;
typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

const int maxRadius = 150;
const int minRadius = 20;

// Set 1 to disable
const int minWhiteThreshold = 255 * 0.30;
const int maxWhiteThreshold = 255 * 1;

const int sizeThreshold = 200;

void PotholeDetector::img_callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_ptr = cv_bridge::toCvCopy(msg, "");
    Mat orig = cv_ptr->image.clone();
    src = cv_ptr->image.clone();

    //Crops the image (removes sky)
    int topCrop = src.rows / 2 - 100;
    cv::Rect myROI(0, topCrop, src.cols, src.rows - topCrop);
    src = src(myROI);

    cvtColor(src, src_gray, CV_BGR2GRAY);

    //Find the mean and stddev of the grayscale image in order to do adaptive thresholding
    Mat mean;
    Mat stddev;
    meanStdDev(src_gray, mean, stddev);

    double thresh = mean.at<double>(0,0) + (stddev.at<double>(0,0) * 2);
    if(thresh > 254) {
        thresh = 254;
    }

    threshold(src_gray, src_gray, thresh, 255, THRESH_BINARY);

    GaussianBlur(src_gray, src_gray, Size(gaussian_size, gaussian_size), 2, 2);

    vector<Vec3f> circles;

    // TODO tune circle radii for actual potholes
    HoughCircles(src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows / 8, 50, 10, minRadius, maxRadius);

    Mat cloudMat = Mat::zeros(orig.rows, orig.cols, CV_32F);
    /// Put the circles into a matrix
    for( size_t i = 0; i < circles.size(); i++ ) {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);

        // If the circle is too close to the top / bottom edges, filter
        if (center.y <= 100 || center.y >= src_gray.rows - 100) {
            continue;
        }

        // If the circle is too close to the left / right edges, filter
        if (center.x <= 200 || center.x >= src_gray.cols - 200) {
            continue;
        }

        int cropSize = (100 / (double) src_gray.rows) * center.y;
        //Get a matrix around circle center
        cv::Rect myROI(center.x - (cropSize * 2), center.y - cropSize, cropSize * 4, cropSize * 2);
        Mat roi = src_gray(myROI);

        // If the sum of all the pixels in the rectangle is less than sumThreshold
        // Then this circle is not encompassing mostly white pixels
        const int sumThreshold = (cropSize * cropSize * 8);
        double sum = cv::sum(roi)[0];
        if (sum < sumThreshold * minWhiteThreshold || sum > sumThreshold * maxWhiteThreshold) {
            continue;
        }

        //circle(cloudMat, center, radius, Scalar(255), 1, 8, 0);
        rectangle(src, myROI, Scalar(255), 2, 8, 0);
        cerr << "" + to_string(cropSize) + " " + to_string(radius) << endl;
        circle(src, center, radius, Scalar(255), 2, 8, 0);

        Point offset(center.x - (cropSize * 2), center.y - cropSize);
        vector<vector<Point>> contours;
        findContours(roi, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE, Point((center.x - (cropSize * 2)), center.y - cropSize));

        // Filter false positives - small contours and cone stripes
        for (vector<vector<Point>>::iterator it = contours.begin(); it != contours.end(); ++it) {
            vector<Point> curCont = *it;
            if (curCont.size() <= sizeThreshold) {
                contours.erase(it);
                --it;
            }
        }

        drawContours(src, contours, -1, Scalar(20, 236, 27), 3, 8);
    }

    cvtColor(src_gray, src_gray, CV_GRAY2BGR);

    cv_bridge::CvImage out_msg;
    out_msg.header   = msg->header;
    out_msg.encoding = msg->encoding;
    out_msg.image    = src;

    cv_ptr->image = src;
    _pothole_filt_img.publish(cv_ptr->toImageMsg());
    _pothole_thres.publish(out_msg.toImageMsg());
    cloud = toPointCloud(cloudMat);
    _pothole_cloud.publish(cloud);
}

PotholeDetector::PotholeDetector(ros::NodeHandle &handle)
    : gaussian_size(7),
      _it(handle),
      tf_listener(handle) {
    _src_img = _it.subscribe("/stereo/right/image_raw", 1, &PotholeDetector::img_callback, this);
    _pothole_filt_img = _it.advertise("/pothole_filt_img", 1);
    _pothole_thres = _it.advertise("/pothole_thres", 1);
    _pothole_cloud = handle.advertise<PCLCloud>("/pothole_cloud", 100);
}

// FIXME does not take into account distance from groud to camera
PointCloud<PointXYZ>::Ptr PotholeDetector::toPointCloud(Mat src) {
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    for(int r = 0; r < src.rows; r++) {
        float *row = src.ptr<float>(r);
        for(int c = 0; c < src.cols; c++) {
            if(row[c] > 0) {
                cloud->points.push_back(PointXYZ(r, c, 0));
            }
        }
    }
	cloud->header.frame_id = "base_footprint";
	return cloud;
}
