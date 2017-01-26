#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <mutex>
#include <opencv/cv.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

const char* const lidar_frame = "/lidar";
const char* const lidar_topic = "/scan/pointcloud";
const char* const camera_frame = "/main_camera_link";
const char* const camera_topic = "/igvc/usb_cam_center/image_raw";
const char* const image_topic = "/scan/image";

std::mutex cloud_mutex;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
std::unique_ptr<tf::TransformListener> tf_listener;
image_transport::Publisher image_pub;

bool tf_seen = false;

void scanCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(cloud_mutex);
    cloud->clear();

    tf::StampedTransform transform;
    if (!tf_seen)
    {
        tf_listener->waitForTransform(camera_frame, lidar_frame, ros::Time(0), ros::Duration(5));
        tf_seen = true;
    }
    tf_listener->lookupTransform(camera_frame, lidar_frame, ros::Time(0), transform);

    pcl_ros::transformPointCloud(*msg, *cloud, transform);
}

void cameraCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    std::lock_guard<std::mutex> lock(cloud_mutex);
    if (cloud->empty())
    {
        return;
    }

    cv_bridge::CvImagePtr input_bridge = cv_bridge::toCvCopy(image_msg, "");
    cv::Mat& image = input_bridge->image;
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(info_msg);

    for (auto point : *cloud)
    {
        cv::Point3d cv_point(point.x, point.y, point.z);
        cv::Point2d cam_point = cam_model.project3dToPixel(cv_point);
        if (cam_point.x < image.cols && cam_point.y < image.rows
            && cam_point.x >= 0 && cam_point.y >= 0)
        {
            cv::Vec3b pixel = image.at<cv::Vec3b>(cam_point);
            pixel.val[0] = 0;
            pixel.val[1] = 0;
            pixel.val[2] = 0;
            image.at<cv::Vec3b>(cam_point) = pixel;
        }
    }
    image_pub.publish(input_bridge->toImageMsg());
}

int main(int argc, char** argv)
{
    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

    ros::init(argc, argv, "scan_to_image");

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    tf_listener.reset(new tf::TransformListener());

    ros::Subscriber scan_sub = nh.subscribe(lidar_topic, 1, scanCallback);
    image_transport::CameraSubscriber cam_sub = it.subscribeCamera(camera_topic, 1, cameraCallback);

    image_pub = it.advertise(image_topic, 1);

    ros::spin();
}
