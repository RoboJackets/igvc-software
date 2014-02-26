#ifndef SIMULATEDGPS_H
#define SIMULATEDGPS_H

#include "GPS.hpp"
#include <boost/thread.hpp>
#include <queue>
#include <common/utils/gpsfilereader.h>

class SimulatedGPS : public GPS
{
public:
    SimulatedGPS(std::string file);

    bool StateIsAvailable();
    GPSData GetState();
    bool isOpen();

    ~SimulatedGPS();

    void setHz(double Hz);

private:
    boost::thread _thread;

    bool _open;

    void threadRun();

    std::queue<GPSData> _data;
    bool _running;

    int _delay;

};

#endif // SIMULATEDGPS_H
