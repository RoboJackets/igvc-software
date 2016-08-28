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
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
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
    LineDetector(ros::NodeHandle &handle, const std::string& topic);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

private:
    cv::Mat src_img, working, fin_img;

    void img_callback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info);
    pcl::PointXYZ PointFromPixel(const cv::Point&, const tf::Transform&);

    pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud(cv::Mat src);

    // ROS COMMUNICATION
    std::string topic;
    image_transport::ImageTransport _it;
    image_transport::Publisher _filt_img;
    image_transport::CameraSubscriber _src_img;
    ros::Publisher _line_cloud;
    tf::TransformListener tf_listener;
    image_geometry::PinholeCameraModel cam;
};
#endif // LINEDETECTOR_H
