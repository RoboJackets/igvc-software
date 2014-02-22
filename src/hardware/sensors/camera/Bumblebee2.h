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


void ProcessFrame(Image* rawImage, void const* otherThings);
void PrintError(FlyCapture2::Error error);

using namespace cv;
using namespace FlyCapture2;

class Bumblebee2 : public StereoSource
{
    public:
        Bumblebee2(string fileName ="/home/robojackets/igvc/2013/software/src/sensors/camera3D/calib/out_camera_data.xml");
        virtual ~Bumblebee2();
        int Run();
        //void LockImages();
        //void UnlockImages();
        static void ptgrey2opencv(FlyCapture2::Image& img, cv::Mat& mat);
        ImageData left();
        ImageData right();
        Mat& LeftMat();
        Mat& RightMat();
        StereoImageData Images();
        void Images(Mat& leftImage, Mat& rightImage);
        int frameCount;
        boost::mutex frameLock;
        FlyCapture2::Camera& Cam();
        Mat correctImage(Mat rawImg);
    private:
        int StartCamera();
        int CloseCamera();
        Mat _cameraMatrix;
        Mat _distCoeffs;
        StereoImageData _images;
        FlyCapture2::Camera _cam;
        //boost::mutex _imagesLock;
};



#endif // BUMBLEBEE2_H
