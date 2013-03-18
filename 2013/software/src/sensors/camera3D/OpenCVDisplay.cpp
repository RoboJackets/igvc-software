#include "OpenCVDisplay.h"

OpenCVDisplay::OpenCVDisplay(Bumblebee2& cam) : LonNewFrame(this), _LeftWindowName("left"), _RightWindowName("right")
{
    cam.onNewData += &LonNewFrame;
    namedWindow(_RightWindowName,1);
    namedWindow(_LeftWindowName,1);
}

void OpenCVDisplay::onNewFrame(StereoPair newFrame)
{
    imshow(_RightWindowName, newFrame.RightImage());
    imshow(_LeftWindowName, newFrame.LeftImage());
}

OpenCVDisplay::~OpenCVDisplay()
{
    //dtor
}
