#pragma once
#include <ros/ros.h>
#include <ros/publisher.h>
#include <flycapture/FlyCapture2.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

class Bumblebee2
{
public:
    Bumblebee2(ros::NodeHandle &handle);
    
    ~Bumblebee2();
    
    bool isOpen();
    
private:

    void startCamera();
    
    void closeCamera();
    
    FlyCapture2::Camera _cam;

    image_transport::ImageTransport _it;
    image_transport::Publisher _left_pub;
    image_transport::Publisher _right_pub;
    ros::Publisher _leftInfo_pub;
    ros::Publisher _rightInfo_pub;

    camera_info_manager::CameraInfoManager cameraManagerLeft;
    camera_info_manager::CameraInfoManager cameraManagerRight;
    
    static void ProcessFrame(FlyCapture2::Image* rawImage, const void* callbackData);
};
