#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <camera_info_manager/camera_info_manager.h>
#include <ros/publisher.h>

ros::Publisher info_pub;
camera_info_manager::CameraInfoManager* cameraManager;

void callback(const sensor_msgs::ImageConstPtr& msg) {
    sensor_msgs::CameraInfo cl = cameraManager->getCameraInfo();
    cl.header.frame_id = "/usb_cam";
    cl.header.stamp = msg->header.stamp;
    cameraManager->setCameraInfo(cl);

    info_pub.publish(cameraManager->getCameraInfo());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "webcam");
    ros::NodeHandle nh;

    cameraManager = new camera_info_manager::CameraInfoManager(ros::NodeHandle("/usb_cam"), "/usb_cam", "file:///home/nareddyt/Desktop/cal.yml");
    info_pub = nh.advertise<sensor_msgs::CameraInfo>("/usb_cam/camera_info", 1);

    ros::Subscriber sub = nh.subscribe("/usb_cam/image_raw", 1, callback);

    ros::spin();
}