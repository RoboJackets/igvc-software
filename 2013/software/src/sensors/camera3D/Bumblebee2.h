#ifndef BUMBLEBEE2_H
#define BUMBLEBEE2_H

#include <opencv2/opencv.hpp>
#include <flycapture/FlyCapture2.h>
#include <boost/thread.hpp>
#include <stdio.h>
#include <unistd.h>
#include <opencv2/highgui/highgui.hpp>

#include "events/Event.hpp"
#include "sensors/camera3D/StereoPair.hpp"
#include "sensors/camera3D/StereoSource.hpp"
#include "sensors/DataStructures/SensorData.h"


void ProcessFrame(Image* rawImage, void const* otherThings);
void PrintError(FlyCapture2::Error error);

using namespace cv;
using namespace FlyCapture2;

class Bumblebee2 : public StereoSource
{
    public:
        Bumblebee2();
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
        int frameCount;
        boost::mutex frameLock;
        FlyCapture2::Camera& Cam();
    private:
        int StartCamera();
        int CloseCamera();
        StereoImageData _images;
        FlyCapture2::Camera _cam;
        //boost::mutex _imagesLock;
};



#endif // BUMBLEBEE2_H
