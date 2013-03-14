#ifndef STEREOPAIR_H
#define STEREOPAIR_H

#include <flycapture/FlyCapture2.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace FlyCapture2;

class StereoPair
{
    public:
        inline StereoPair()
        {
        }
        //TODO this might be causing a memory leak, check to make sure this is the correct way to initialize
        inline StereoPair(Mat left, Mat right)
        {
            _leftImage = left;
            _rightImage = right;
        }
        inline Mat& LeftImage()
        {
            return _leftImage;
        }
        inline Mat& RightImage()
        {
            return _rightImage;
        }
    private:
        Mat _leftImage;
        Mat _rightImage;
};


#endif // STEREOPAIR_H
