#include "sensors/ardupilot/Ardupilot.hpp"
#include <string.h>
#include <iostream>
#include <sstream>
#include <sensors/timing.h>
#include <sensors/DataStructures/IMUData.hpp>
#include <common/utils/StringUtils.hpp>

using namespace std;

Ardupilot::Ardupilot()
 : ardupilotPort("/dev/ttyIMU", 115200),
   LonNewSerialLine(this)
{
    ardupilotPort.onNewLine += &LonNewSerialLine;
    ardupilotPort.startEvents();
}

Ardupilot::~Ardupilot()
{
    ardupilotPort.onNewLine -= &LonNewSerialLine;
    ardupilotPort.stopEvents();
    ardupilotPort.close();
}

void Ardupilot::onNewSerialLine(string line)
{
    if(line[0] == 'A')
    {
        vector<string> tokens = split(line, ' ');
        IMURawData data;
        data.roll = atof(tokens[1].c_str());
        data.pitch = atof(tokens[2].c_str());
        data.heading = atof(tokens[3].c_str());
        data.accelX = atof(tokens[4].c_str());
        data.accelY = atof(tokens[5].c_str());
        data.accelZ = atof(tokens[6].c_str());
        onNewRawData(data);
    }
}
