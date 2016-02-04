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
const int sizeThreshold = 200;
const int rOrange = 190;
const int gOrange = 60;
const int bOrange = 35;

double radians(double degrees)
{
    return degrees / 180.0 * M_PI;
}

int getDiff(int a, int b) {
    return abs(a - b);
}

void PotholeDetector::img_callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_ptr = cv_bridge::toCvCopy(msg, "");
    src = cv_ptr->image.clone();
    int height = src.rows;
    int width = src.cols;

    //Crops the image (removes sky)
    cv::Rect myROI(0, src.rows/2 - 100, src.cols, src.rows/2 - 50);
    src = src(myROI);

    cvtColor(src, src_gray, CV_BGR2GRAY);

    /*
     * Find the mean and stddev of the grayscale
     * image in order to do adaptive thresholding
     */
    Mat mean;
    Mat stddev;
    meanStdDev(src_gray, mean, stddev);

    double thresh = mean.at<double>(0,0) + (stddev.at<double>(0,0) * 2);
    if(thresh > 254)
    {
        thresh = 254;
    }

    threshold(src_gray, src_gray, thresh, 255, THRESH_BINARY);

    GaussianBlur(src_gray, src_gray, Size(gaussian_size, gaussian_size),
            100, 100);

    vector<vector<Point>> contours;
    findContours(src_gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

    // Filter false positives - small contours and cone stripes
    for (vector<vector<Point>>::iterator it = contours.begin();
            it != contours.end(); ++it) {
        if ((*it).size() > sizeThreshold) {
            // Find center in x frame
            int minY = height;
            int minX = width;
            int maxY = 0;
            int maxX = 0;
            for (Point p : *it) {
                int x = p.x;
                if (x > maxX) {
                    maxX = x;
                }
                if (x < minX) {
                    minX = x;
                }
            }
            int centerX = (minX + maxX) / 2;

            // Find centers of top and bottom edges
            for (Point p : *it) {
                int x = p.x;
                int y = p.y;
                if (x == centerX && y > maxY) {
                    maxY = y;
                }
                if (x == centerX && y < minY) {
                    minY = y;
                }
            }

            // Check if contour is within bounds of image
            if (minY - 35 >= 0) {

                // Average rgb values above and below the contour
                int blueAbove = 0;
                int greenAbove = 0;
                int redAbove = 0;
                int blueBelow = 0;
                int greenBelow = 0;
                int redBelow = 0;
                Vec3b currentPixel;
                for (int j = 5; j < 36; j++) {
                    currentPixel = src.at<Vec3b>(minY - j, centerX);
                    blueAbove += currentPixel[0];
                    greenAbove += currentPixel[1];
                    redAbove += currentPixel[2];
                    currentPixel = src.at<Vec3b>(maxY + j, centerX);
                    blueBelow += currentPixel[0];
                    greenBelow += currentPixel[1];
                    redBelow += currentPixel[2];
                }
                blueAbove /= 30;
                greenAbove /= 30;
                redAbove /= 30;
                blueBelow /= 30;
                greenBelow /= 30;
                redBelow /= 30;

                // Filter out contours with orange above and below
                if (getDiff(redAbove, rOrange) < 50
                        && getDiff(greenAbove, gOrange) < 50
                        && getDiff(blueAbove, bOrange) < 50
                        && getDiff(redBelow, rOrange) < 50
                        && getDiff(greenBelow, gOrange) < 50
                        && getDiff(blueBelow, bOrange) < 50) {
                    contours.erase(it);
                    --it;
                }

                // Delete if the contour itself is orange
                Vec3b intensity = src.at<Vec3b>((minY + maxY) / 2, centerX);
                uchar blue = intensity.val[0];
                uchar green = intensity.val[1];
                uchar red = intensity.val[2];
                if (getDiff(red, rOrange) < 50 && getDiff(green, gOrange) < 50
                        && getDiff(blue, bOrange) < 50) {
                    contours.erase(it);
                    --it;
                }
            } else {
                contours.erase(it);
                --it;
            }
        } else {
            contours.erase(it);
            --it;
        }
    }

    Mat cloudMat = Mat::zeros(height, width, CV_32F);

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
