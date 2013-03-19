#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <flycapture/FlyCapture2.h>
#include "Bumblebee2.h"
#include "sensors/camera3D/OpenCVDisplay.h"
#include "sensors/camera3D/StereoVidMaker.h"
using namespace cv;



int main()
{


    Bumblebee2 thisguy;
    std::cout << "got through Bumblebee Constructor" << std::endl;
    OpenCVDisplay Disp(&thisguy);
    std::string fName = "/home/robojackets/Desktop/this.mpeg";
    int nFrame = 300;
    StereoVidMaker maker (&thisguy, nFrame, fName);
    while(thisguy.Running())
    {
    }

    //thisguy.StartCamera();
    //thisguy.run();

}

/*
int main(int, char**)
{
    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    Mat edges;
    namedWindow("edges",1);
    namedWindow("edges2",2);
    for(;;)
    {
        Mat frame;
        Mat frame2;
        cap >> frame;
        imshow("edges", frame);

        for (int row =0; row < frame.rows; row++)
        {
            uchar* p = frame.ptr(row);
            for(int col =1; col < frame.cols; col+=4)
            {
                char tmp = p[col];
                p[col] = p[col-1];
                p[col-1] = tmp;
            }
        }

        imshow("edges2", frame);
        //cap >> frame; // get a new frame from camera

        //cvtColor(frame, edges, CV_BGR2GRAY);

        //GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
        //Canny(edges, edges, 0, 30, 3);

        //imshow("edges2", frame2);
        if(waitKey(30) >= 0) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
*/
