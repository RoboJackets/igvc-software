#ifndef BUMBLEBEE2_H
#define BUMBLEBEE2_H

#include <opencv2/opencv.hpp>
#include <flycapture/FlyCapture2.h>
#include <boost/thread.hpp>
#include <stdio.h>
#include <unistd.h>
#include <opencv2/highgui/highgui.hpp>

#include <hardware/sensors/camera/StereoPair.hpp>
#include <hardware/sensors/camera/StereoSource.hpp>
#include <common/datastructures/SensorData.hpp>
#include "common/utils/ImageUtils.h"


void ProcessFrame(FlyCapture2::Image* rawImage, void const* otherThings);
void PrintError(FlyCapture2::Error error);

class Bumblebee2 : public StereoSource
{
    public:
        Bumblebee2(std::string fileName ="/home/robojackets/igvc/2013/software/src/sensors/camera3D/calib/out_camera_data.xml");
        virtual ~Bumblebee2();
        int Run();
        //void LockImages();
        //void UnlockImages();
        static void ptgrey2opencv(FlyCapture2::Image& img, cv::Mat& mat);
        ImageData left();
        ImageData right();
        cv::Mat& LeftMat();
        cv::Mat& RightMat();
        StereoImageData Images();
        void Images(cv::Mat& leftImage, cv::Mat& rightImage);
        int frameCount;
        boost::mutex frameLock;
        FlyCapture2::Camera& Cam();
        cv::Mat correctImage(cv::Mat rawImg);
    private:
        int StartCamera();
        int CloseCamera();
        cv::Mat _cameraMatrix;
        cv::Mat _distCoeffs;
        StereoImageData _images;
        FlyCapture2::Camera _cam;
        //boost::mutex _imagesLock;
};



#endif // BUMBLEBEE2_H
