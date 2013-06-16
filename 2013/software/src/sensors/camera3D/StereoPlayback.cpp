#include "StereoPlayback.h"
#include <boost/thread/condition_variable.hpp>
#include <unistd.h>

StereoPlayback::StereoPlayback(std::string leftVideo, std::string rightVideo, int fps, string fileName, bool undistort) : _leftVid(leftVideo), _rightVid(rightVideo),
    _framesPerSecond(fps), _playbackThread(&StereoPlayback::Run, this), _undistort(undistort)
{
  FileStorage fs(fileName, FileStorage::READ); // Read the settings
  fs["Camera_Matrix"] >> _cameraMatrix;
  fs["Distortion_Coefficients"] >> _distCoeffs;
  fs.release();
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
            LockRunning();
            Running(false);
            UnlockRunning();
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
