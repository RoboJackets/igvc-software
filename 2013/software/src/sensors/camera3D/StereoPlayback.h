#ifndef STEREOPLAYBACK_H
#define STEREOPLAYBACK_H

#include "sensors/camera3D/StereoSource.hpp"
#include "sensors/DataStructures/StereoImageData.hpp"


class StereoPlayback : public StereoSource
{
    public:
        StereoPlayback(std::string leftVideo, std::string rightVideo, int fps=20);
        void Run();
        virtual ~StereoPlayback();

    private:
        VideoCapture _leftVid;
        VideoCapture _rightVid;
        int _framesPerSecond;
        StereoImageData _images;
        boost::thread _playbackThread;
};

#endif // STEREOPLAYBACK_H
