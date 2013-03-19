#include "sensors/camera3D/StereoVidMaker.h"

StereoVidMaker::StereoVidMaker(Bumblebee2* cam, int frameCount, string videoName): LonNewFrame(this), _frameCount(frameCount)
{
    (*cam).onNewData += &LonNewFrame;
    _writer.open((const std::string&)videoName, CV_FOURCC('P','I','M','1'), (int)20, cv::Size(1024,768), true);

}
void StereoVidMaker::onNewFrame(StereoPair newPair)
{
    if (_frameCount <= 0)
    {

    }
    else
    {
        _writer.write((const cv::Mat&) newPair.LeftImage());
        _frameCount--;
    }
}

StereoVidMaker::~StereoVidMaker()
{
    //dtor
}
