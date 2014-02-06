#ifndef SIMULATEDGPS_H
#define SIMULATEDGPS_H

#include "GPS.hpp"
#include <boost/thread.hpp>
#include <queue>
#include <common/utils/gpsfilereader.h>

namespace IGVC {
namespace Sensors {

class SimulatedGPS : public GPS
{
public:
    SimulatedGPS(std::string file);

    bool StateIsAvailable();
    GPSData GetState();
    GPSData GetStateAtTime(timeval time);
    bool isOpen();

    ~SimulatedGPS();

private:
    boost::thread _thread;

    void threadRun();

    std::queue<GPSData> _data;
    bool _running;

};

}
}
#endif // SIMULATEDGPS_H
