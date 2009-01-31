#ifndef CVCAM_H_
#define CVCAM_H_

/*
 * This file is an interface to a USB camera, using OpenCV
 *   by Chris McClanahan
 *
 */

//#include <stdio.h>
#include <highgui.h>




class CVcam
{
public:
    // constructor
    CVcam();
    // destructor
    virtual ~CVcam();
    // the connection to the camera
    CvCapture* capture;
    // flag for init capture connection
    int camconnected;

    // returns 1 if connected to camera capture
    int isValid();
    // grabs raw opencv image (into global visCvRaw)
    int GrabCvImage();
    // grabs raw image and converts it into (global) visRaw
    void GrabBuffer2DImage();
    // sets up the camera capture object
    int connect(int deviceID = 0, const char* filename = NULL);
    // starts image grab thread (not currently used)
    void startImageGrabThread(void* obj);

    // loads values/settings for camera
    void loadSettings();

    // testing usb camera functions (not currently used)
    int testwebcamandconvertloop();




};

#endif /*CVCAM_H_*/
