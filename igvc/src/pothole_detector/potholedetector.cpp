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

constexpr double radians(double degrees)
{
    return degrees / 180.0 * M_PI;
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

    GaussianBlur(src_gray, src_gray, Size(gaussian_size, gaussian_size), 2, 2);

    vector<Vec3f> circles;
    // TODO tune circle radii for actual potholes
    HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 50, 10, 30, 50);

    for (std::vector<Vec3f>::const_iterator i = circles.begin(); i != circles.end(); ++i) {
        std::cout << *i << ' ';
    }

    Mat cloudMat = Mat::zeros(orig.rows, orig.cols, CV_32F);
    /// Put the circles into a matrix
    for( size_t i = 0; i < circles.size(); i++ )
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));

        //If circle center is too close to bottom and right edge of image
        if(center.x+30 >= src_gray.cols || center.y+30 >= src_gray.rows)
        {
            continue;
        }

        //Get 60x60 matrix around circle center
        cv::Rect myROI(center.x - 30 >= 0 ? center.x-30 : 0, center.y - 30 >= 0 ? center.y-30 : 0, 60, 60);
        Mat roi = src_gray(myROI);

        //If the sum of all the pixels in the 60x60 mat is less than sumThreshold
        //Then this circle is not encompassing mostly white pixels
        double sum = cv::sum(roi)[0];
        if(sum < sumThreshold)
        {
            continue;
        }

        int radius = cvRound(circles[i][2]);
        circle(cloudMat, center, radius, Scalar(255), 1, 8, 0);
        circle(src, center, radius, Scalar(255), 1, 8, 0);
    }

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