#ifndef ARDUPILOT_H
#define ARDUPILOT_H

#include "serial/ASIOSerialPort.h"
#include "sensors/DataStructures/IMUData.hpp"

using namespace std;

class Ardupilot
{
    public:
        Ardupilot();
        ~Ardupilot();
        void update();
        void write(char a);
        Event<IMUData> onNewIMUData;

    private:
        double timeLast;
        double timeCurrent;
        double heading;
        double speed;
        double positionX;
        double positionY;
        double deltat;
        ASIOSerialPort ardupilotPort;
};

#endif // ARDUPILOT_H
