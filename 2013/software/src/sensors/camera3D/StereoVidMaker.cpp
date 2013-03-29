#include "sensors/camera3D/StereoVidMaker.h"

StereoVidMaker::StereoVidMaker(StereoSource& source, string videoName,int frameCount, int frameRate, bool video): LonNewFrame(this), _totalFrames(frameCount), _source(source),
_leftWriter(), _rightWriter(), _nFrames(0), _video(video), _name(videoName)
{
    _source.onNewData += &LonNewFrame;
    const std::string leftVidName = videoName + "_left" +".mpeg";
    const std::string rightVidName = videoName + "_right" + ".mpeg";
    _leftWriter.open((const std::string&)leftVidName, CV_FOURCC('P','I','M','1'), frameRate, cv::Size(1024,768), true);
    _rightWriter.open((const std::string&)rightVidName, CV_FOURCC('P','I','M','1'), frameRate, cv::Size(1024,768), true);

}

void StereoVidMaker::onNewFrame(StereoPair& newPair)
{
    if (_nFrames < totalFrames)
    {
        if (_video)
        {
            addFrame(newPair);
        }
        else
        {
            takeStereoImage(newPair);
        }
    }
    else
    {
        _source.onNewData -= &LonNewFrame;
    }
}

void StereoVidMaker::addFrame(StereoPair& newPair)
{
    std::cout << "printing frame" << std::endl;
    //_rightWriter.write((const cv::Mat&) newPair.RightImage());
    //_leftWriter.write((const cv::Mat&) newPair.LeftImage());
    _rightWriter << newPair.RightImage();
    _leftWriter << newPair.LeftImage();
    _nFrames++;
}


void StereoVidMaker::takeStereoImage(StereoPair& newPair)
{
    const std::string leftImageName = videoName + _nFrames + "_left" +".jpg";
    const std::string rightImageName = videoName + _nFrames + "_right" + ".jpg";
    imwrite(leftImageName, newPair.LeftImage());
    imwrite(rightImageName, newPair.RightImage());
}

StereoVidMaker::~StereoVidMaker()
{
    //dtor
}
