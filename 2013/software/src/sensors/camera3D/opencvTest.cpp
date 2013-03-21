#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <flycapture/FlyCapture2.h>
#include "Bumblebee2.h"
#include "sensors/camera3D/OpenCVDisplay.h"
#include "sensors/camera3D/StereoVidMaker.h"
#include "sensors/camera3D/StereoSource.hpp"
#include "sensors/camera3D/StereoPlayback.h"
using namespace cv;



int main()
{

    std::string fName = "/home/robojackets/Desktop/this";
    std::string lName, rName;
    lName = fName + "_left" + ".mpeg";
    rName = fName + "_right" + ".mpeg";
    StereoPlayback thisguy(lName, rName, 20);
    Bumblebee2 thisguy;


    OpenCVDisplay Disp(thisguy);

    //
    /* //initialize video maker
    std::string fName = "/home/robojackets/Desktop/this";
    //std::string fName = "/home/alex/Desktop/vid1";
    int nFrame = 300;
    StereoVidMaker maker(thisguy, nFrame, fName);
    */

    while(thisguy.Running())
    {
    }
}
