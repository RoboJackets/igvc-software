#include <competitionControl/CompetitionController.h>
#include <sensors/GPS/HemisphereA100GPS.h>
#include <sensors/lidar/NAV200.h>
#include <sensors/camera3D/Bumblebee2.h>
#include <sensors/camera2D/laneDetection/CameraListener.hpp>
#include <actuators/motors/MotorDriver/MotorEncoderDriver2013.h>
#include <iostream>

using namespace IGVC::Control;
using namespace IGVC::Sensors;
using namespace std;

int main()
{
    cout << "Loading waypoints..." << endl;

    WaypointReader waypointReader;
    waypointReader.LoadWaypoints("/home/robojackets/practiceCourse.txt");

    cout << "Connecting to GPS..." << endl;
    HemisphereA100GPS GPS;

    cout << "Connecting to motor driver..." << endl;

    MotorEncoderDriver2013 driver;

    cout << "Connecting to camera..." << endl;

    Bumblebee2 camera;

    cout << "Connecting to lidar..." << endl;

    NAV200 lidar;

    cout << "Initializing listener..." << endl;

    CameraListener camList(&camera, &lidar);

    cout << "Initializing controller..." << endl;

    CompetitionController controller(&GPS,
                                     &camList.OnNewData,
                                     &waypointReader,
                                     &driver);

    cout << "Running..." << endl;

    while(controller.isRunning()) { }

    cout << "done" << endl;

    return 0;
}
