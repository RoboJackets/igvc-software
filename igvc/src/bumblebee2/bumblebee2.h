#pragma once
#include <ros/ros.h>
#include <ros/publisher.h>
#include <flycapture/FlyCapture2.h>

class Bumblebee2
{
public:
    Bumblebee2();
    
    ~Bumblebee2();
    
private:

    void startCamera();
    
    void closeCamera();
    
    FlyCapture2::Camera _cam;
    
    static void ProcessFrame(FlyCapture2::Image* rawImage, const void* callbackData);
};
