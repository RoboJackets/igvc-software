#include <string.h>
#include <serial/ASIOSerialPort.h>
#include <iostream>
#include <sensors/ardupilot/Ardupilot.hpp>

//GPSAccuracy IMUData::ArduPilotDefault= GPSAccuracy(.001, .001, 1, .1);

class IMUListener
{
public:
    IMUListener(Ardupilot* device):
      LonNewIMUData(this),
      LonNewRawData(this)
    {
        device->onNewData += &LonNewIMUData;
        device->onNewRawData += &LonNewRawData;
    }

private:
    void onNewIMUData(IMUData data) {
        cout <<  data.Yaw() << endl;
    }
    LISTENER(IMUListener, onNewIMUData, IMUData);
    void onNewRawData(IMURawData data) {
        //std::cout << data.roll << std::endl;
    }
    LISTENER(IMUListener, onNewRawData, IMURawData);
};

int main()
{
    Ardupilot ardupilot;
    IMUListener listener(&ardupilot);
    std::cout<<"Running"<<endl;
    while(true){ };
}
