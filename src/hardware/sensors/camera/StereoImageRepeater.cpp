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
        onNewLeftImage(_left);
        onNewRightImage(_right);
        try {
            boost::this_thread::interruption_point();
        } catch (...) {
            return;
        }

        usleep(5000000); //500
    }
}

bool StereoImageRepeater::IsConnected(){
    return true;
}

StereoImageRepeater::~StereoImageRepeater()
{
    _thread.interrupt();
    _thread.join();
}


