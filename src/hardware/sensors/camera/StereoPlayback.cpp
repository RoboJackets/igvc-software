#include "StereoPlayback.h"
#include <boost/thread/condition_variable.hpp>
#include <unistd.h>

using namespace cv;

StereoPlayback::StereoPlayback(std::string leftVideo, std::string rightVideo, int fps, std::string fileName, bool undistort) : _leftVid(leftVideo), _rightVid(rightVideo),
    _framesPerSecond(fps), _undistort(undistort), _playbackThread(&StereoPlayback::Run, this)
{
    if(_undistort)
    {
        FileStorage fs(fileName, FileStorage::READ); // Read the settings
        fs["Camera_Matrix"] >> _cameraMatrix;
        fs["Distortion_Coefficients"] >> _distCoeffs;
        fs.release();
    }
}

void StereoPlayback::Run()
{
    int waitTime = 1000000/_framesPerSecond;
    bool lsuccess, rsuccess;
    while(Running())
    {
        usleep(waitTime); //TODO Ensure this function call is non-blocking(gives up the processor during sleep) or replace it with something that is

        Mat left, right;
        lsuccess = _leftVid.grab();
        _leftVid.retrieve(left);
        rsuccess = _rightVid.grab();
        if (lsuccess && rsuccess)
        {
            _rightVid.retrieve(right);

            if (_undistort)
            {
              left = correctImage(left);
              right = correctImage(right);
            }

            _images = StereoImageData(left, right);
            onNewData(_images);
        }
        else
        {
            _leftVid.set(CV_CAP_PROP_POS_FRAMES , 0);
            _rightVid.set(CV_CAP_PROP_POS_FRAMES, 0);
//            LockRunning();
//            Running(false);
//            UnlockRunning();
        }
    }
}

Mat StereoPlayback::correctImage(Mat img)
{
  return correctDistortion(img,  _cameraMatrix, _distCoeffs);
}


StereoPlayback::~StereoPlayback()
{
    //dtor
}
