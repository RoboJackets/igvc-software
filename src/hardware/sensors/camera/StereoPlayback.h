#ifndef STEREOPLAYBACK_H
#define STEREOPLAYBACK_H

#include <hardware/sensors/camera/StereoSource.hpp>
#include <common/datastructures/StereoImageData.hpp>
#include <common/utils/ImageUtils.h>


class StereoPlayback : public StereoSource
{
    public:
        StereoPlayback(std::string leftVideo, std::string rightVideo, int fps=20,
                       string fileName="/home/robojackets/igvc/2013/software/src/sensors/camera3D/calib/out_camera_data.xml",
                       bool undistort=true);
        void Run();
        Mat correctImage(Mat);
        virtual ~StereoPlayback();

    private:
        VideoCapture _leftVid;
        VideoCapture _rightVid;
        int _framesPerSecond;
        StereoImageData _images;
        Mat _cameraMatrix;
        Mat _distCoeffs;
        bool _undistort;
        boost::thread _playbackThread;

};

#endif // STEREOPLAYBACK_H
