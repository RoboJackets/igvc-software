#include <competitionControl/CompetitionController.h>
#include <sensors/GPS/HemisphereA100GPS.h>
#include <sensors/lidar/NAV200.h>
#include <sensors/camera3D/Bumblebee2.h>

using namespace IGVC::Control;
using namespace IGVC::Sensors;

int main()
{
    WaypointReader waypointReader;
    waypointReader.LoadWaypoints("");

    Event<void*> myEvent;

    CompetitionController controller(new HemisphereA100GPS(),
                                     myEvent,
                                     new Ardupilot(),
                                     &waypointReader);

    while(controller.isRunning()) { }

    return 0;
}
