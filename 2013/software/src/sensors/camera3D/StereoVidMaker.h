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
        StereoVidMaker(Bumblebee2* cam, int frameCount, String videoName);
        virtual ~StereoVidMaker();
        void onNewFrame(StereoPair);
        LISTENER(StereoVidMaker,onNewFrame, StereoPair);
    private:
        int _frameCount;
        VideoWriter _writer;


};

#endif // STEREOVIDMAKER_H
