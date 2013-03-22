#include "OpenCVDisplay.h"

OpenCVDisplay::OpenCVDisplay(StereoSource& source) : LonNewFrame(this), _LeftWindowName("left"), _RightWindowName("right"), _source(source)
{
    _source.onNewData += &LonNewFrame;
    namedWindow(_RightWindowName,1);
    namedWindow(_LeftWindowName,1);
}

void OpenCVDisplay::onNewFrame(StereoPair& newFrame)
{
    _source.LockImages();
    Mat newLeft, newRight;
    newRight = newFrame.RightImage().clone();
    newLeft = newFrame.LeftImage().clone();

    //imshow(_RightWindowName, newFrame.RightImage());
    //imshow(_LeftWindowName, newFrame.LeftImage());

    imshow(_RightWindowName,newRight);
    imshow(_LeftWindowName, newLeft);
    _source.UnlockImages();
    if(waitKey(20) >= 0)
    {
        _source.LockRunning();
        _source.Running(false);
        _source.UnlockRunning();
    }
}

OpenCVDisplay::~OpenCVDisplay()
{
    //dtor
}
