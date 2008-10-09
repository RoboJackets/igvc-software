#ifndef GUPPYCAM_H_
#define GUPPYCAM_H_


#include "DCam.h"
#include "DCam_Config.h"
#include <dc1394/control.h>


/*
 * This file provides a wrapper for DCam.h for the AVT Guppy Camera
 *   by: Chris McClanahan
 *
 */

class GuppyCam {
public:
    GuppyCam();
    virtual ~GuppyCam();

    // camera objects
    Camera::DCam *_dcam;
    dc1394feature_t _feature;
    dc1394camera_t *_camera;

    // connects to 1394 camera
    int connect();

    // flag for init capture connection
    int camconnected;

    // returns 1 if connected to camera capture
    int isValid();

    // grabs raw opencv image (into global visCvRaw)
    bool GrabCvImage();

    // grabs raw image and converts it into (global) visRaw
    void GrabBuffer2DImage();

    // camera settings
    void setGain(int value);
    void setShutter(int value);
    void setGamma(int value);
    void setWhiteBalance(uint32_t red, uint32_t blue);

    // sets gain and shutter to auto-adjust
    void setAllAuto();


    // loads values/settings for camera
    void loadSettings();

};

#endif /*GUPPYCAM_H_*/
