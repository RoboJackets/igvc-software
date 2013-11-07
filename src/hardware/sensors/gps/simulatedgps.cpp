#include "simulatedgps.h"
#include <iostream>
#include <fstream>
#include <common/utils/StringUtils.hpp>
#include <common/logger/logger.h>

namespace IGVC {
namespace Sensors {


SimulatedGPS::SimulatedGPS(std::string file)
    : _index(0),
      _running(true)
{
    loadFile(file);
    _thread = boost::thread(boost::bind(&SimulatedGPS::threadRun, this));
}

bool SimulatedGPS::StateIsAvailable()
{
    return true;
}

GPSData SimulatedGPS::GetState()
{
    _index++;
    _index %= _data.size();
    return _data.at(_index);
}

GPSData SimulatedGPS::GetStateAtTime(timeval)
{
    return GetState();
}

void SimulatedGPS::threadRun()
{
    while(_running)
    {
        onNewData(_data.at(_index));
        _index++;
        _index %= _data.size();
        sleep(1);
    }
}

void SimulatedGPS::loadFile(std::string file)
{
    using namespace std;
    string line = "";
    ifstream infile;
    infile.open(file.c_str());

    if(!infile.is_open())
    {
        stringstream msg;
        msg << "Could not open file : " << file;
        Logger::Log(LogLevel::Error, msg.str());
        _data.push_back(GPSData());
        return;
    }

    while(!infile.eof())
    {
        getline(infile, line);
        if(line.length() > 0)
        {
            vector<string> tokens = split(line, ',');
            GPSData newData;
            newData.Lat(atof(tokens.at(0).c_str()));
            newData.Long(atof(tokens.at(1).c_str()));
            _data.push_back(newData);
        }
    }

    infile.close();
}

SimulatedGPS::~SimulatedGPS()
{
    _running = false;
    _thread.join();
}

}
}
