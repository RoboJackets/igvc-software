#ifndef LINEDETECTOR_H
#define LINEDETECTOR_H
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <flycapture/FlyCapture2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <vector>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#define KERNAL_COUNT 8

class LineDetector
{
public:
    LineDetector(ros::NodeHandle &handle);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

private:
    void initLineDetection();

    void DetectLines(int lineThickness);
    void WhitenessFilter(cv::Mat& hsv_image, cv::Mat& result);
    void MultiplyByComplements(cv::Mat* images, cv::Mat* complements, cv::Mat* results);
    void EnforceLength(cv::Mat& img, int length);



    // line thickness in pixels
    int lineThickness;
    // line threshold to continue
    int lineLengthThreshold;

    const int linewidthpixels = 13;

    cv::Mat src_img;
    cv::Mat dst_img;
    cv::Mat fin_img;
    cv::Mat kernels[KERNAL_COUNT];
    cv::Mat kernelComplements[KERNAL_COUNT];
    cv::Mat working;
    cv::Mat kernelResults[KERNAL_COUNT];
    cv::Mat complementResults[KERNAL_COUNT];

    void img_callback(const sensor_msgs::ImageConstPtr& msg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud(cv::Mat src);

    // ROS COMMUNICATION
    image_transport::ImageTransport _it;
    image_transport::Publisher _filt_img;
    image_transport::Subscriber _src_img;
    ros::Publisher _line_cloud;
    tf::TransformListener tf_listener;
};
#endif // LINEDETECTOR_H
