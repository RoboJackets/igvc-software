#include <competitionControl/CompetitionController.h>
#include <sensors/GPS/HemisphereA100GPS.h>
#include <sensors/lidar/NAV200.h>
#include <sensors/camera3D/Bumblebee2.h>
#include <sensors/camera2D/laneDetection/CameraListener.hpp>
#include <actuators/motors/MotorDriver/MotorEncoderDriver2013.h>

using namespace IGVC::Control;
using namespace IGVC::Sensors;

int main()
{
    WaypointReader waypointReader;
    waypointReader.LoadWaypoints("");

    MotorEncoderDriver2013 driver;

    Bumblebee2 camera;

    NAV200 lidar;

    CameraListener camList(&camera, &lidar);



    CompetitionController controller(new HemisphereA100GPS(),
                                     &camList.OnNewData,
                                     new Ardupilot(),
                                     &waypointReader,
                                     &driver);

    while(controller.isRunning()) { }

    return 0;
}
