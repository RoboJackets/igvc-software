#include "StereoImageRepeater.h"

StereoImageRepeater::StereoImageRepeater(std::string pathLeft, std::string pathRight)
{
    _left = cv::imread(pathLeft);
    _right = cv::imread(pathRight);
    _thread =  boost::thread(boost::bind(&StereoImageRepeater::thread_run, this));
}

void StereoImageRepeater::thread_run()
{
    while(true)
    {
        onNewData(StereoImageData(_left, _right));
        usleep(500);
    }
}

StereoImageRepeater::~StereoImageRepeater()
{

}
