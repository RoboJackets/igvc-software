#ifndef ARDUPILOT_H
#define ARDUPILOT_H

#include <hardware/serial/ASIOSerialPort.h>
#include <hardware/sensors/DataStructures/IMUData.hpp>
#include <hardware/sensors/IMU/IMU.h>

using namespace std;

class Ardupilot : public IMU
{
    public:
        Ardupilot();
        ~Ardupilot();
        Event<IMUData> onNewIMUData;

    private:
        ASIOSerialPort ardupilotPort;

        void onNewSerialLine(string line);
        LISTENER(Ardupilot, onNewSerialLine, string);
};

#endif // ARDUPILOT_H
