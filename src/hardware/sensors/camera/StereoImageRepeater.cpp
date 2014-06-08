#include "StereoImageRepeater.h"
#include <common/config/configmanager.h>
#include <common/logger/logger.h>
#include <QDir>

StereoImageRepeater::StereoImageRepeater(std::string pathLeft, std::string pathRight)
{
    if(QFile(pathLeft.c_str()).exists() && QFile(pathRight.c_str()).exists())
    {
        _left = cv::imread(pathLeft);
        _right = cv::imread(pathRight);
        _thread =  boost::thread(boost::bind(&StereoImageRepeater::thread_run, this));
    }
    else
    {
        Logger::Log(LogLevel::Error, "Image repeater could not load one or both of the images.");
    }
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
        _fps = ConfigManager::Instance().getValue("ImageRepeater", "FPS", 3);
        usleep(1000000/_fps);
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


