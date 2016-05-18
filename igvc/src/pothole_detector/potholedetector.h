#ifndef POTHOLEDETECTOR_H
#define POTHOLEDETECTOR_H

#include <ros/ros.h>
#include <ros/publisher.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <vector>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
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

class PotholeDetector {
public:
    PotholeDetector(ros::NodeHandle &handle);
    PointCloud<pcl::PointXYZ>::Ptr cloud;

private:
    void camera_info_callback(const sensor_msgs::CameraInfoPtr& msg);
    void img_callback(const sensor_msgs::ImageConstPtr& msg);

    PointCloud<PointXYZ>::Ptr toPointCloud(vector<vector<Point>> &contours, int height, int width);

    /**
     * @brief gaussian_size The size of the Gaussian blur. The bigger the greater the blur
     * @note Must be odd!
     */
    const int gaussian_size;

    /**
     * @brief src contains the original, unprocessed image
     */
    //cv::Mat src;
    /**
     * @brief dst contains the new, processed image that isolates the lines
     */
    //cv::Mat *dst;

    // ROS COMMUNICATION
    image_transport::ImageTransport _it;
    image_transport::Publisher _pothole_filt_img;
    image_transport::Publisher _pothole_thres;
    image_transport::Subscriber _src_img;
    ros::Subscriber _camera_info;
    ros::Publisher _pothole_cloud;
    tf::TransformListener tf_listener;

    Mat src;
    Mat src_gray;
};
#endif // POTHOLEDETECTOR_H
