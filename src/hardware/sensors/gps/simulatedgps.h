#ifndef SIMULATEDGPS_H
#define SIMULATEDGPS_H

#include "GPS.hpp"
#include <boost/thread.hpp>
#include <vector>

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

    void loadFile(std::string file);

    std::vector<GPSData> _data;
    int _index;
    bool _running;

};

}
}
#endif // SIMULATEDGPS_H
