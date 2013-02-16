#include "sensors/lidar/SimulatedLidar.h"

namespace IGVC
{
namespace Sensors
{

SimulatedLidar::SimulatedLidar()
{
    _running = true;
    _thread = boost::thread(boost::bind(&SimulatedLidar::thread_run, this));
}

void SimulatedLidar::thread_run()
{
    while(_running)
    {
        sleep(1);
        LidarState state;
        for(int i = 0; i < 1024; i++)
        {
            LidarPoint &p = state.points[i];
            p.valid = true;
            p.angle = i * ( ( M_PI * 2.0 ) / 1024);
            p.distance = 1 / (sin(p.angle) + cos(p.angle));
//            p.distance = i/1.024;
        }
        onNewData(state);
    }
}

SimulatedLidar::~SimulatedLidar()
{
    _running = false;
    _thread.join();
}

LidarState SimulatedLidar::GetState()
{
    return LidarState();
}

LidarState SimulatedLidar::GetStateAtTime(timeval time)
{
    return LidarState();
}

bool SimulatedLidar::StateIsAvailable()
{
    return false;
}

}
}
