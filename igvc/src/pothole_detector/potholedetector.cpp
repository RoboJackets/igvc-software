#include "potholedetector.h"
#include <opencv2/video/video.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <pcl_ros/point_cloud.h>

using namespace std;
using namespace cv;
using namespace pcl;

cv_bridge::CvImagePtr cv_ptr;
typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

void PotholeDetector::img_callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_ptr = cv_bridge::toCvCopy(msg, "");
    Mat src = (cv_ptr->image).clone();

    /// Convert it to gray
    Mat src_gray;

    circle(src, Point(50,50), 40, Scalar(200,0,0), 3, 8, 0);
    cvtColor(src, src_gray, CV_BGR2GRAY);


    /// Reduce the noise so we avoid false circle detection
    GaussianBlur(src_gray, src_gray, Size(gaussian_size, gaussian_size), 2, 2);

    /// Apply the Hough Transform to find the circles
    vector<Vec3f> circles;
    HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 100, 100, 20, 200);
    /// Draw the circles detected
    for( size_t i = 0; i < circles.size(); i++ )
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        circle(src, center, 3, Scalar(0,255,0), -1, 8, 0);
        // circle outline
        circle(src, center, radius, Scalar(0,0,255), 3, 8, 0);
    }

    cv_ptr->image = src;
    _pothole_filt_img.publish(cv_ptr->toImageMsg());
}

PotholeDetector::PotholeDetector(ros::NodeHandle &handle)
    : gaussian_size(7),
      _it(handle),
      tf_listener(handle)
{
    //_src_img = _it.subscribe("/left/image_rect_color", 1, &PotholeDetector::img_callback, this);
    _src_img = _it.subscribe("/usb_cam/image_raw", 1, &PotholeDetector::img_callback, this);
    _pothole_filt_img = _it.advertise("/pothole_filt_img", 1);
    _pothole_cloud = handle.advertise<PCLCloud>("/pothole_cloud", 100);
}


//void PotholeDetector::detectObstacle(int row, int col, cv::Mat* dst)
//{
//    Vec3b p = dst->at<Vec3b>(row,col);
//    int row2 = row;
//    int col2 = col;
//
//    //While the pixel is still orange, turn it black
//    //Then on to the next one, by row
//    while (p[2] > 100 /*&& (p[1] < 150 || p[1] > 250)*/){
//        dst->at<Vec3b>(row2, col)[0] = 0;
//        dst->at<Vec3b>(row2, col)[1] = 0;
//        dst->at<Vec3b>(row2, col)[2] = 0;
//        p = dst->at<Vec3b>(++row2, col);
//    }
//    p = dst->at<Vec3b>(row,col);
//
//    //While the pixel is still orange, turn it black
//    //Then on to the next one, by column
//    while (p[2] > 100 /*&& (p[1] < 150 || p[1] > 250)*/){
//        dst->at<Vec3b>(row, col2)[0] = 0;
//        dst->at<Vec3b>(row, col2)[1] = 0;
//        dst->at<Vec3b>(row, col2)[2] = 0;
//        p = dst->at<Vec3b>(row, ++col2);
//    }
//
//    //Turn everything in that block we just found black
//    for(int i = row+1; i<row2;i++){
//        for (int j = col+1; j<col2; j++){
//            dst->at<Vec3b>(i,j)[0] = 0;
//            dst->at<Vec3b>(i,j)[1] = 0;
//            dst->at<Vec3b>(i,j)[2] = 0;
//        }
//    }
//}
//
//void PotholeDetector::blackoutSection(int rowl, int rowu, int coll, int colu)
//{
//    for (int i=rowl;i<=rowu;i++)
//    {
//        for (int j = coll; j<=colu; j++)
//        {
//            dst->at<Vec3b>(i,j)[0] = 0;
//            dst->at<Vec3b>(i,j)[1] = 0;
//            dst->at<Vec3b>(i,j)[2] = 0;
//        }
//    }
//}
//
//PointCloud<PointXYZ>::Ptr PotholeDetector::toPointCloud(Mat src)
//{
//    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
//    tf::StampedTransform transform;
//    tf_listener.lookupTransform("/base_footprint", "/camera", ros::Time(0), transform);
//    double roll, pitch, yaw;
//    tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
//    auto origin_z = transform.getOrigin().getZ();
//    auto origin_y = transform.getOrigin().getY();
//    auto HFOV = radians(66.0);
//    auto VFOV = radians(47.6);
//    pitch = -roll; // Because conventions are different and I'm in the middle of comp, and give me a break.
//    for(int r = src.rows/2; r < src.rows; r++)
//    {
//        uchar *row = src.ptr<uchar>(r);
//        for(int c = 0; c < src.cols; c++)
//        {
//            if(row[c] > 0)
//            {
//                auto pitch_offset = ((float)(r-src.rows/2) / src.rows) * VFOV;
//                auto y = origin_z /tan(pitch + pitch_offset) + origin_y;
//
//                auto theta = ((float)(c-src.cols/2) / src.cols) * HFOV;
//                auto x = y * tan(theta);
//
//                cloud->points.push_back(PointXYZ(x, y, 0));
//            }
//        }
//    }
//	cloud->header.frame_id = "base_footprint";
//	return cloud;
//}