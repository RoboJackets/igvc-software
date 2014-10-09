#include "simulatedgps.h"
#include <iostream>
#include <fstream>
#include <common/utils/StringUtils.hpp>
#include <common/logger/logger.h>

SimulatedGPS::SimulatedGPS(std::string file) : _running(true)
{
    try {
        GPSFileReader::read(file, _data);
        _open = true;
    } catch (GPSFileNotFoundException) {
        Logger::Log(LogLevel::Error, "[SimulatedGPS] Could not find file: " + file);
        _open = false;
    } catch (GPSFileFormatException) {
        Logger::Log(LogLevel::Error, "[SimulatedGPS] File " + file + " is not formatted correctly.");
        _open = false;
    }
    _delay = 100000;
    _thread = boost::thread(boost::bind(&SimulatedGPS::threadRun, this));
}

bool SimulatedGPS::StateIsAvailable()
{
    return _open && _data.size() > 0;
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

bool SimulatedGPS::isWorking()
{
    return _open && _data.size() > 0;
}

void SimulatedGPS::threadRun()
{
    while(_running)
    {
        if(_data.size() > 0)
        {
            onNewData(GetState());
        }
        usleep(_delay);
    }
}

SimulatedGPS::~SimulatedGPS()
{
    _running = false;
    _thread.join();
}

void SimulatedGPS::setHz(double Hz)
{
    // 1 million / ( 1 / x seconds) = x million microseconds
    _delay = 1000000/Hz;
}
