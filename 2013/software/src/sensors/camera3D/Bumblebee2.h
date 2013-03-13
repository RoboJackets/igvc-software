#ifndef BUMBLEBEE2_H
#define BUMBLEBEE2_H

#include <opencv2/opencv.hpp>
#include "events/Event.hpp"
#include <flycapture/FlyCapture2.h>
#include <boost/thread.hpp>

using namespace cv;
using namespace FlyCapture2;

class ImagePair
{
    public:
        inline ImagePair()
        {
        }
        //TODO this might be causing a memory leak, check to make sure this is the correct way to initialize
        inline ImagePair(Mat left, Mat right)
        {
            _leftImage = left;
            _rightImage = right;
        }
        inline Mat& leftImage()
        {
            return _leftImage;
        }
        inline Mat& rightImage()
        {
            return _rightImage;
        }
    private:
        Mat _leftImage;
        Mat _rightImage;
};


class Bumblebee2
{
    public:
        Bumblebee2();
        virtual ~Bumblebee2();
        Event<ImagePair> onNewData;
        int Run();
    private:
        int StartCamera();
        int CloseCamera();
        void ptgrey2opencv(FlyCapture2::Image& img, cv::Mat& mat);
        void PrintError( FlyCapture2::Error error );
        ImagePair _images;
        FlyCapture2::Camera _cam;
        boost::mutex _imagesLock;
        bool _running;
};


#endif // BUMBLEBEE2_H
