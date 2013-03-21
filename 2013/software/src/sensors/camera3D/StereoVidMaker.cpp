#include "sensors/camera3D/StereoVidMaker.h"

StereoVidMaker::StereoVidMaker(StereoSource& source, int frameCount, string videoName): LonNewFrame(this), _frameCount(frameCount), _source(source)
{
    source.onNewData += &LonNewFrame;
    const std::string leftVidName = videoName + "_left" +".mpeg";
    const std::string rightVidName = videoName + "_right" + ".mpeg";
    _leftWriter.open((const std::string&)leftVidName, CV_FOURCC('P','I','M','1'), (int)20, cv::Size(1024,768), true);
    _rightWriter.open((const std::string&)rightVidName, CV_FOURCC('P','I','M','1'), (int)20, cv::Size(1024,768), true);

}

void StereoVidMaker::onNewFrame(StereoPair newPair)
{
    if (_frameCount <= 0)
    {

    }
    else
    {
        std::cout << "printing frame" << std::endl;
        _rightWriter.write((const cv::Mat&) newPair.LeftImage());
        _leftWriter.write((const cv::Mat&) newPair.LeftImage());
        _frameCount--;
    }
}

StereoVidMaker::~StereoVidMaker()
{
    //dtor
}
