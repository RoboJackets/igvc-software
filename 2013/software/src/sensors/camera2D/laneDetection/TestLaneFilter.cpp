#include <sensors/lidar/NAV200.h>
#include <sensors/camera3D/Bumblebee2.h>
#include <sensors/camera2D/laneDetection/CameraListener.hpp>
#include <iostream>
#include <sensors/camera3D/StereoPlayback.h>

using namespace IGVC::Sensors;
using namespace std;

class CameraListenerListener
{
public:
    CameraListenerListener(CameraListener* camList)
        : LOnNewFileteredFrame(this)
    {
        _camList = camList;
        _camList->OnNewFilteredFrame += &LOnNewFileteredFrame;
    }

private:
    CameraListener* _camList;

    void OnNewFileteredFrame(Mat frame)
    {
        imshow("Frame", frame);
    }
    LISTENER(CameraListenerListener, OnNewFileteredFrame, Mat);
};

int main()
{

    //Bumblebee2 camera;

    StereoPlayback camera("/home/robojackets/Desktop/camTesting/data/CompCourse_left0.mpeg", "/home/robojackets/Desktop/camTesting/data/CompCourse_right0.mpeg");

    cout << "Connecting to lidar..." << endl;

    //NAV200 lidar;

    cout << "Initializing listener..." << endl;

    CameraListener camList(&camera, 0);

    CameraListenerListener camListList(&camList);


    cout << "Running..." << endl;

    while(true) { }

    cout << "done" << endl;

    return 0;
}
