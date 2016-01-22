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
//Used to threshold the sum of a 60x60 matrix whose values are between 0-255
//Want mostly (>50%) white pixels (values around >200) in the matrix
//60x60x200ish = 720,000/2 = ~400000
const int sumThreshold = 400000;
const int sizeThreshold = 200;

constexpr double radians(double degrees)
{
    return degrees / 180.0 * M_PI;
}

constexpr int getDiff(int a, int b) {
    return abs(a - b);
}

void PotholeDetector::img_callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_ptr = cv_bridge::toCvCopy(msg, "");
    Mat orig = cv_ptr->image.clone();
    src = cv_ptr->image.clone();

    //Crops the image (removes sky)
    cv::Rect myROI(0, src.rows/2 - 100, src.cols, src.rows/2 - 50);
    src = src(myROI);

    cvtColor(src, src_gray, CV_BGR2GRAY);

    //Find the mean and stddev of the grayscale image in order to do adaptive thresholding
    Mat mean;
    Mat stddev;
    meanStdDev(src_gray, mean, stddev);

    double thresh = mean.at<double>(0,0) + (stddev.at<double>(0,0) * 2);
    if(thresh > 254)
    {
        thresh = 254;
    }

    threshold(src_gray, src_gray, thresh, 255, THRESH_BINARY);

    GaussianBlur(src_gray, src_gray, Size(gaussian_size, gaussian_size), 100, 100);

    vector<vector<Point>> contours;
    findContours(src_gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

    // Filter smaller contours
    for (unsigned int i = 0; i < contours.size(); i++) {
        vector<Point> curCont = contours[i];
        if (curCont.size() <= sizeThreshold) {
            contours.erase(contours.begin() + i);
            i--;
        }
    }

    // Get min / max Y and X
    int minY = 10000;
    int minX;
    int maxY = 0;
    int maxX;
    for (unsigned int i = 0; i < contours.size(); i++) {
        for (Point p : contours[i]) {
            int y = p.y;
            if (y > maxY) {
                maxY = y;
                maxX = p.x;
            } else if (y < minY) {
                minY = y;
                minX = p.x;
            }
        }

        // Delete if there is orange below or above
        Vec3b intensityAbove = orig.at<Vec3b>(minX, minY - 5);
        uchar greenAbove = intensityAbove.val[1];
        uchar redAbove = intensityAbove.val[2];
        Vec3b intensityBelow = orig.at<Vec3b>(maxX, maxY + 5);
        uchar greenBelow = intensityBelow.val[1];
        uchar redBelow = intensityBelow.val[2];
        if (getDiff(greenAbove, 125) > 50 && getDiff(redAbove, 240) > 50 && getDiff(greenBelow, 125) > 50 && getDiff(redBelow, 240) > 50) {    // Play with these thresholds
            contours.erase(contours.begin() + i);
            i--;
        }

        // Delete if the contour itself is orange
        Vec3b intensity = orig.at<Vec3b>((minX + maxX) / 2, (minY + maxY) / 2);
        uchar green = intensity.val[1];
        uchar red = intensity.val[2];
        if (getDiff(green, 125) > 50 && getDiff(red, 240) > 50) {    // Play with these thresholds
            contours.erase(contours.begin() + i);
            i--;
        }
    }

    /// Draw contours 
    drawContours(src, contours, -1, Scalar(255), 2, 8);

    Mat cloudMat = Mat::zeros(orig.rows, orig.cols, CV_32F);
    cvtColor(src_gray, src_gray, CV_GRAY2BGR);

    cv_bridge::CvImage out_msg;
    out_msg.header   = msg->header;
    out_msg.encoding = msg->encoding;
    out_msg.image    = src_gray;

    cv_ptr->image = src;
    _pothole_filt_img.publish(cv_ptr->toImageMsg());
    _pothole_thres.publish(out_msg.toImageMsg());
    cloud = toPointCloud(cloudMat);
    _pothole_cloud.publish(cloud);
}

PotholeDetector::PotholeDetector(ros::NodeHandle &handle)
    : gaussian_size(7),
      _it(handle),
      tf_listener(handle)
{
    _src_img = _it.subscribe("/left/image_rect_color", 1, &PotholeDetector::img_callback, this);
    _pothole_filt_img = _it.advertise("/pothole_filt_img", 1);
    _pothole_thres = _it.advertise("/pothole_thres", 1);
    _pothole_cloud = handle.advertise<PCLCloud>("/pothole_cloud", 100);
}

PointCloud<PointXYZ>::Ptr PotholeDetector::toPointCloud(Mat src)
{
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    for(int r = 0; r < src.rows; r++)
    {
        float *row = src.ptr<float>(r);
        for(int c = 0; c < src.cols; c++)
        {
            if(row[c] > 0)
            {
                cloud->points.push_back(PointXYZ(r, c, 0));
            }
        }
    }
	cloud->header.frame_id = "base_footprint";
	return cloud;
}