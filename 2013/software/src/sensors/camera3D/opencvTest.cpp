#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <flycapture/FlyCapture2.h>
#include "Bumblebee2.h"
#include "sensors/camera3D/OpenCVDisplay.h"
#include "sensors/camera3D/StereoVidMaker.h"
#include "sensors/camera3D/StereoSource.hpp"
#include "sensors/camera3D/StereoPlayback.h"
#include "sensors/camera3D/featureFinder.h"


using namespace cv;

/*
int main()
{
    std::string  left = "/home/alex/Desktop/IGVC/2013/software/trainingSets/cone/set2/11123906-0-Left.bmp";
    std::string  right = "/home/alex/Desktop/IGVC/2013/software/trainingSets/cone/set2/11123906-0-Right.bmp";
    Mat L, R, D, LG, RG;
    namedWindow("this",0);
    L = imread(left);
    R = imread(right);
    cvtColor(L, LG, CV_BGR2GRAY );
    cvtColor(R, RG, CV_BGR2GRAY );
    StereoBM S;
    //S.init(0);

    Ptr<CvStereoBMState> def = S.state;
    def->preFilterSize=5;
    def->preFilterCap=6;
    def->SADWindowSize = 45;
    def->minDisparity = 0;
    def->numberOfDisparities = 80;
    def->textureThreshold = 0;
    def->uniquenessRatio =2;
    def->speckleWindowSize = 0;
    def->speckleRange = 60;


    S(LG,RG,D,CV_16S);
    Mat dispD;
    //convertScaleAbs(D, dispD, 2);
    double minVal, maxVal;
    minMaxLoc(D, &minVal, &maxVal);
    D.convertTo(dispD, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));
    imshow("this", dispD);
    if(waitKey(20000) >= 0);
}
*/


int main()
{

    //std::string fName = "/home/robojackets/Desktop/this";
    std::string fName = "/home/alex/Desktop/IGVC/2013/software/trainingSets/this";
    std::string lName, rName;
    lName = fName + "_left" + ".mpeg";
    rName = fName + "_right" + ".mpeg";
    StereoPlayback thisguy(lName, rName, 20);

    //Bumblebee2 thisguy;

    //OpenCVDisplay Disp(thisguy);
    featureFinder he(thisguy);

    //initialize video maker

    std::string saveName = "/home/robojackets/Desktop/camTesting/highRes1024packet20fps";


    //int nFrame = 100;
    //StereoVidMaker maker(thisguy, nFrame, saveName);


    while(thisguy.Running())
    {
    }
}
