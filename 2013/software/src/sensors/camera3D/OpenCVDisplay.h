#ifndef OPENCVDISPLAY_H
#define OPENCVDISPLAY_H

#include "sensors/camera3D/StereoPair.hpp"
#include "events/Delegate.hpp"
#include "sensors/camera3D/StereoSource.hpp"

class OpenCVDisplay
{
    public:
        OpenCVDisplay(StereoSource&);
        virtual ~OpenCVDisplay();
        void onNewFrame(StereoPair newFrame);
        LISTENER(OpenCVDisplay, onNewFrame, StereoPair);

    private:
        string _LeftWindowName;
        string _RightWindowName;
        StereoSource& _source;
};

#endif // OPENCVDISPLAY_H
