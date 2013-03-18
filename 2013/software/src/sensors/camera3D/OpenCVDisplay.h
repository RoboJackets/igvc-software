#ifndef OPENCVDISPLAY_H
#define OPENCVDISPLAY_H

#include "sensors/camera3D/StereoPair.hpp"
#include "events/Delegate.hpp"
#include "sensors/camera3D/Bumblebee2.h"

class OpenCVDisplay
{
    public:
        OpenCVDisplay(Bumblebee2&);
        virtual ~OpenCVDisplay();
        void onNewFrame(StereoPair newFrame);
        LISTENER(OpenCVDisplay, onNewFrame, StereoPair);

    private:
        string _LeftWindowName;
        string _RightWindowName;
};

#endif // OPENCVDISPLAY_H
