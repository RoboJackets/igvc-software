#include "simulatedgps.h"
#include <iostream>
#include <fstream>
#include <common/utils/StringUtils.hpp>
#include <common/logger/logger.h>

namespace IGVC {
namespace Sensors {


SimulatedGPS::SimulatedGPS(std::string file) : _running(true)
{
    GPSFileReader::read(file, _data);
    _thread = boost::thread(boost::bind(&SimulatedGPS::threadRun, this));
}

bool SimulatedGPS::StateIsAvailable()
{
    return true;
}

GPSData SimulatedGPS::GetState()
{
    if(_data.size() == 0)
        return GPSData();


    _data.push(_data.front()); //Readd point to back of queue to allow cycling through log file
    GPSData temp = _data.front();
    _data.pop();
    return temp;
}

GPSData SimulatedGPS::GetStateAtTime(timeval)
{
    return GetState();
}

bool SimulatedGPS::isOpen()
{
    return _data.size() > 0;
}

void SimulatedGPS::threadRun()
{
    while(_running)
    {
        if(_data.size() > 0)
        {
            onNewData(GetState());
            sleep(1);
        }
    }
}

SimulatedGPS::~SimulatedGPS()
{
    _running = false;
    _thread.join();
}

}
}
