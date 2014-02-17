#include "hardware/sensors/IMU/Ardupilot.h"
#include <string.h>
#include <iostream>
#include <sstream>
#include <common/utils/timing.h>
#include <common/utils/StringUtils.hpp>
#include <common/logger/logger.h>

using namespace std;

Ardupilot::Ardupilot()
 : ardupilotPort("/dev/ttyIMU", 115200)
{
    connect(&ardupilotPort, SIGNAL(onNewLine(std::string)), this, SLOT(onNewSerialLine(std::string)));
    ardupilotPort.startEvents();
}

Ardupilot::~Ardupilot()
{
    disconnect(&ardupilotPort, SIGNAL(onNewLine(std::string)), this, SLOT(onNewSerialLine(std::string)));
    ardupilotPort.stopEvents();
    ardupilotPort.close();
}

void Ardupilot::onNewSerialLine(string line)
{
    if(line[0] == 'A')
    {
        vector<string> tokens = split(line, ' ');
        if(tokens.size() == 7)
        {
            IMUData data;
            data.Roll = atof(tokens[1].c_str());
            data.Pitch = atof(tokens[2].c_str());
            data.Yaw = atof(tokens[3].c_str());
            data.X = atof(tokens[4].c_str());
            data.Y = atof(tokens[5].c_str());
            data.Z = atof(tokens[6].c_str());

            onNewData(data);
        }
        else
        {
            stringstream msg;
            msg << "IMU could not parse line : " << line;
            Logger::Log(LogLevel::Warning, msg.str());
        }
    }
}

bool Ardupilot::isWorking()
{
    return ardupilotPort.isConnected();
}
