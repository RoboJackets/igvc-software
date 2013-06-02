#include "OpenCVDisplay.h"

OpenCVDisplay::OpenCVDisplay(StereoSource& source, string path) : LonNewFrame(this), _LeftWindowName("left"), _RightWindowName("right"),
_capturePath(path), _imNum(0), _source(source)
{
    _source.onNewData += &LonNewFrame;
    namedWindow(_RightWindowName,1);
    namedWindow(_LeftWindowName,1);
}

void OpenCVDisplay::onNewFrame(StereoImageData newFrame)
{
    _source.LockImages();
    Mat newLeft, newRight;
    newRight = newFrame.rightMat().clone();
    newLeft = newFrame.leftMat().clone();

    imshow(_RightWindowName,newRight);
    imshow(_LeftWindowName, newLeft);
    _source.UnlockImages();

    char dat = waitKey(10);
    if (dat == ' ')
    {
      std::stringstream leftname, rightname;
      leftname << _capturePath << "_left" << _imNum << ".jpg";
      imwrite(leftname.str(), newFrame.leftMat());
      //name.clear();
      rightname << _capturePath << "_right" << _imNum << ".jpg";
      imwrite(rightname.str(), newFrame.rightMat());
      _imNum++;

    }
    else if(dat >= 0)
    {
        std::cout << "non space button pushed, closing" << std::endl;
        _source.LockRunning();
        _source.Running(false);
        _source.UnlockRunning();
    }
}



OpenCVDisplay::~OpenCVDisplay()
{
    //dtor
}
