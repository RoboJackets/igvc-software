#ifndef STEREOPAIR_H
#define STEREOPAIR_H


#include <flycapture/FlyCapture2.h>
#include <opencv2/opencv.hpp>

class StereoPair
{
    public:
        inline StereoPair()
        {
        }
        //TODO this might be causing a memory leak, check to make sure this is the correct way to initialize
        inline StereoPair(cv::Mat left, cv::Mat right)
        {
            _leftImage = left;
            _rightImage = right;
        }
        inline cv::Mat& LeftImage()
        {
            return _leftImage;
        }
        inline cv::Mat& RightImage()
        {
            return _rightImage;
        }
    private:
        cv::Mat _leftImage;
        cv::Mat _rightImage;
};


#endif // STEREOPAIR_H
