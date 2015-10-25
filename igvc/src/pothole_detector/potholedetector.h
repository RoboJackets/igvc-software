#ifndef POTHOLEDETECTOR_H
#define POTHOLEDETECTOR_H
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

class PotholeDetector
{
public:
    PotholeDetector(ros::NodeHandle &handle);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

private:
    void img_callback(const sensor_msgs::ImageConstPtr& msg);

    //pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud(cv::Mat src);

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
    image_transport::Subscriber _src_img;
    ros::Publisher _pothole_cloud;
    tf::TransformListener tf_listener;
};
#endif // POTHOLEDETECTOR_H
