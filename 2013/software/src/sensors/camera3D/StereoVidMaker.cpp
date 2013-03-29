#include "sensors/camera3D/StereoVidMaker.h"

StereoVidMaker::StereoVidMaker(StereoSource& source, string videoName,int frameCount, int frameRate=10): LonNewFrame(this), _frameCount(frameCount), _source(source),
_leftWriter(), _rightWriter()
{
    _source.onNewData += &LonNewFrame;
    const std::string leftVidName = videoName + "_left" +".mpeg";
    const std::string rightVidName = videoName + "_right" + ".mpeg";
    _leftWriter.open((const std::string&)leftVidName, CV_FOURCC('P','I','M','1'), frameRate, cv::Size(1024,768), true);
    _rightWriter.open((const std::string&)rightVidName, CV_FOURCC('P','I','M','1'), frameRate, cv::Size(1024,768), true);

}

void StereoVidMaker::onNewFrame(StereoPair& newPair)
{
    if (_frameCount <= 0)
    {
        _source.onNewData -= &LonNewFrame;
    }
    else
    {
        std::cout << "printing frame" << std::endl;
        //_rightWriter.write((const cv::Mat&) newPair.RightImage());
        //_leftWriter.write((const cv::Mat&) newPair.LeftImage());
        _rightWriter << newPair.RightImage();
        _leftWriter << newPair.LeftImage();
        _frameCount--;
    }
}

StereoVidMaker::~StereoVidMaker()
{
    //dtor
}
