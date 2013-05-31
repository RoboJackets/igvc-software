#ifndef OPENCVDISPLAY_H
#define OPENCVDISPLAY_H

#include "sensors/camera3D/StereoPair.hpp"
#include "events/Delegate.hpp"
#include "sensors/camera3D/StereoSource.hpp"

#include <stdlib.h>

class OpenCVDisplay
{
    public:
        OpenCVDisplay(StereoSource& source, string path = "/home/alex/Desktop/img");
        virtual ~OpenCVDisplay();
        void onNewFrame(StereoImageData newFrame);
        LISTENER(OpenCVDisplay, onNewFrame, StereoImageData);

    private:
        string _LeftWindowName;
        string _RightWindowName;
        string _capturePath;
        int _imNum;
        StereoSource& _source;
};

#endif // OPENCVDISPLAY_H
