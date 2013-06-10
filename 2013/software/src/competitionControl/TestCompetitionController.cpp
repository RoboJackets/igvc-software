#include <mapping/LidarToPointCloudConverter.hpp>
#include <mapping/PointCloudAdder.hpp>
#include <competitionControl/CompetitionController.h>
#include <sensors/GPS/HemisphereA100GPS.h>
#include <sensors/lidar/NAV200.h>
#include <sensors/camera3D/Bumblebee2.h>
#include <sensors/camera2D/laneDetection/CameraListener.hpp>
#include <actuators/motors/MotorDriver/MotorEncoderDriver2013.h>
#include <iostream>
#include <sensors/camera3D/StereoPlayback.h>
#include <sensors/ardupilot/Ardupilot.hpp>
#include <actuators/lights/LightController.hpp>

using namespace IGVC::Control;
using namespace IGVC::Sensors;
using namespace std;

int main()
{
    LightController lights;
lights.SetSafetyLightState(Blinking);

    cout << "Loading waypoints..." << endl;

    WaypointReader waypointReader;
    waypointReader.LoadWaypoints("/home/robojackets/practiceCourse.txt");

    cout << "Connecting to GPS..." << endl;
    HemisphereA100GPS GPS;

    cout << "Connecting to IMU..." << endl;
    Ardupilot imu;

    cout << "Connecting to motor driver..." << endl;

    MotorEncoderDriver2013 driver;

    cout << "Connecting to camera..." << endl;

    Bumblebee2 camera;

    //StereoPlayback camera("/home/robojackets/Desktop/camTesting/data/CompCourse_left0.mpeg", "/home/robojackets/Desktop/camTesting/data/CompCourse_right0.mpeg");

    cout << "Connecting to lidar..." << endl;

    //NAV200 lidar;

    //0LidarToPointCloudConverter LTPCC(0);

    cout << "Initializing listener..." << endl;

    CameraListener camList(&camera, 0);

    //PointCloudAdder PCA(&LTPCC.OnNewData, &camList.OnNewData);

    cout << "Initializing controller..." << endl;

    CompetitionController controller(&GPS,
                                     &imu,
                                     &camList.OnNewData,
                                     &waypointReader,
                                     &driver);

    cout << "Running..." << endl;

    while(controller.isRunning()) { }

    lights.SetSafetyLightState(Solid);

    cout << "done" << endl;

    return 0;
}
