#include <string.h>
#include "serial/ASIOSerialPort.h"
#include <iostream>
#include "sensors/ardupilot/Ardupilot.hpp"

GPSAccuracy IMUData::ArduPilotDefault= GPSAccuracy(.001, .001, 1, .1);

class IMUListener
{
public:
    IMUListener(Ardupilot* device):
      LonNewIMUData(this)
    {
        device->onNewIMUData += &LonNewIMUData;
    }

private:
    void onNewIMUData(IMUData data) {
        std::cout << "Heading:" << data.Heading() << "\tSpeed:" << data.Speed() << std::endl;
    }
    LISTENER(IMUListener, onNewIMUData, IMUData);

};

int main()
{
    Ardupilot ardupilot;
    IMUListener listener(&ardupilot);
    std::cout<<"Running"<<endl;
    while(true){ };
//    sleep(1);
//    while(true)
//    {
//        ardupilot.update();
//        usleep(50000);
//    }
}
