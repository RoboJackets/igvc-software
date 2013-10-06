#ifndef ARDUPILOT_H
#define ARDUPILOT_H

#include <hardware/serial/ASIOSerialPort.h>
#include <hardware/sensors/DataStructures/IMUData.hpp>
#include <hardware/sensors/IMU/IMU.h>

using namespace std;

struct IMURawData
{
    double roll;
    double pitch;
    double heading;
    double accelX;
    double accelY;
    double accelZ;
};

class Ardupilot : public IMU
{
    public:
        Ardupilot();
        ~Ardupilot();
        Event<IMUData> onNewIMUData;
        Event<IMURawData> onNewRawData;

    private:
        ASIOSerialPort ardupilotPort;

        void onNewSerialLine(string line);
        LISTENER(Ardupilot, onNewSerialLine, string);
};

#endif // ARDUPILOT_H
