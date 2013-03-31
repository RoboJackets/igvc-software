#ifndef BUMBLEBEE2_H
#define BUMBLEBEE2_H

#include <opencv2/opencv.hpp>
#include <flycapture/FlyCapture2.h>
#include <boost/thread.hpp>
#include "events/Event.hpp"
#include "sensors/camera3D/StereoPair.hpp"
#include "sensors/camera3D/StereoSource.hpp"

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
        Mat& Left();
        Mat& Right();
        StereoPair& Images();
        int frameCount;
        boost::mutex frameLock;
        FlyCapture2::Camera& Cam();
    private:
        int StartCamera();
        int CloseCamera();
        StereoPair _images;
        FlyCapture2::Camera _cam;
        //boost::mutex _imagesLock;
};



#endif // BUMBLEBEE2_H
