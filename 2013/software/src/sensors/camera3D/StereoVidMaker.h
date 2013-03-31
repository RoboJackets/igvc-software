#ifndef STEREOVIDMAKER_H
#define STEREOVIDMAKER_H
#include "events/Delegate.hpp"
#include "sensors/camera3D/StereoPair.hpp"
#include "sensors/camera3D/Bumblebee2.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>



using namespace cv;

class StereoVidMaker
{
    public:
        StereoVidMaker(StereoSource& source, std::string videoName, int frameCount, int frameRate=10, bool video=true); //videoName must NOT have an extension, it will be saved as MPEG
        virtual ~StereoVidMaker();
        void onNewFrame(StereoPair&);
        LISTENER(StereoVidMaker,onNewFrame, StereoPair&);
    private:
        void addFrame(StereoPair&);
        void takeStereoImage(StereoPair&);
        VideoWriter _leftWriter;
        VideoWriter _rightWriter;
        StereoSource& _source;
        int _nFrames;
        int _totalFrames;
        bool _video;
        string _name;


};

#endif // STEREOVIDMAKER_H
