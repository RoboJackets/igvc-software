#include "hardware/sensors/IMU/Ardupilot.h"
#include <string.h>
#include <iostream>
#include <sstream>
#include <common/utils/timing.h>
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
        IMURawData rdata;
        rdata.roll = atof(tokens[1].c_str());
        rdata.pitch = atof(tokens[2].c_str());
        rdata.heading = atof(tokens[3].c_str());
        rdata.accelX = atof(tokens[4].c_str());
        rdata.accelY = atof(tokens[5].c_str());
        rdata.accelZ = atof(tokens[6].c_str());
        onNewRawData(rdata);

        IMUData data(rdata.roll, rdata.pitch, rdata.heading);
        onNewData(data);
    }
}
