#include "simulatedgps.h"
#include <iostream>
#include <fstream>
#include <common/utils/StringUtils.hpp>
#include <common/logger/logger.h>

SimulatedGPS::SimulatedGPS(std::string file) : _running(true)
{
    _moduleName = "GPS";
    try {
        GPSFileReader::read(file, _data);
        std::cout << _data[0].LatVar() << std::endl;
        _open = true;
    } catch (GPSFileNotFoundException) {
        Logger::Log(LogLevel::Error, "[SimulatedGPS] Could not find file: " + file);
        _open = false;
    } catch (GPSFileFormatException) {
        Logger::Log(LogLevel::Error, "[SimulatedGPS] File " + file + " is not formatted correctly.");
        _open = false;
    }
    _delay = 100000;
    _index = 0;
    _thread = boost::thread(boost::bind(&SimulatedGPS::threadRun, this));
}

bool SimulatedGPS::StateIsAvailable()
{
    return _open && _data.size() > 0;
}

const GPSData &SimulatedGPS::GetState()
{
    if(_data.size() == 0)
        return GPSData();

    _index++;
    _index %= _data.size();

//    std::cout << _data[_index].LatVar() << std::endl;

    return _data[_index];
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
            GPSData data = GetState();
            data.setTimeMicroSeconds(std::chrono::high_resolution_clock::now().time_since_epoch() / std::chrono::microseconds(1));
            onNewData(data);
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
