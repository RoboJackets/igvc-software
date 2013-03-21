#include "StereoPlayback.h"
#include <boost/thread/condition_variable.hpp>
#include <unistd.h>

StereoPlayback::StereoPlayback(std::string leftVideo, std::string rightVideo, int fps=20) : _leftVid(leftVideo), _rightVid(rightVideo), _framesPerSecond(fps)
{
}

void StereoPlayback::Run()
{
    int waitTime = 1000000/_framesPerSecond;
    while(Running())
    {
        usleep(waitTime); //TODO Ensure this function call is non-blocking or replace it with something that is
        Mat left, right;
        left = _leftVid.grab();
        right = _rightVid.grab();
        onNewData(StereoPair(left, right));
    }
}

StereoPlayback::~StereoPlayback()
{
    //dtor
}
