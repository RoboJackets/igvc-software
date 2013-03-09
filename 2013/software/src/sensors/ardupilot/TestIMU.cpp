#include <string.h>
#include "serial/ASIOSerialPort.h"
#include <iostream>
#include "sensors/ardupilot/Ardupilot.hpp"

GPSAccuracy IMUData::ArduPilotDefault= GPSAccuracy(.001, .001, 1, .1);

int main()
{
    Ardupilot ardupilot;
    std::cout<<"Running"<<endl;
    sleep(1);
    while(true)
    {
        ardupilot.update();
        usleep(50000);
    }
}
