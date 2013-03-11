#include "sensors/ardupilot/Ardupilot.hpp"
#include <string.h>
#include <iostream>
#include <sstream>
#include "sensors/timing.h"
#include "sensors/DataStructures/IMUData.hpp"

using namespace std;

Ardupilot::Ardupilot():
ardupilotPort("/dev/ttyIMU", 115200)
{
    timeLast = seconds_since_IGVCpoch();
}

Ardupilot::~Ardupilot()
{
    ardupilotPort.close();
}

void Ardupilot::update()
{
    string data = ardupilotPort.readln();
    istringstream iss(data);

    string check;
    iss>>check;

    if(check.compare("A") == 0)
    {
        timeCurrent = seconds_since_IGVCpoch();
        string headingS;
        iss>>headingS;
        heading = atof(headingS.c_str());

        string speedS;
        iss>>speedS;
        speed = atof(speedS.c_str());

        string positionXS;
        iss>>positionXS;
        positionX = atof(positionXS.c_str());

        string positionYS;
        iss>>positionYS;
        positionY = atof(positionYS.c_str());

        deltat = timeCurrent-timeLast;

        cout<<heading<<"\t"<<speed<<"\t"<<positionX<<"\t"<<positionY<<"\t"<<deltat<<endl;

        onNewIMUData(IMUData(positionX, positionY, heading, speed, deltat));

        timeLast = timeCurrent;
    }
}

void Ardupilot::write(char a)
{
    std::ostringstream s;
    s<<a;
    ardupilotPort.write(s.str());
}
