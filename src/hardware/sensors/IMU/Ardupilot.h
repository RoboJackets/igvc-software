#ifndef ARDUPILOT_H
#define ARDUPILOT_H

#include <hardware/serial/ASIOSerialPort.h>
#include <common/datastructures/IMUData.hpp>
#include <hardware/sensors/IMU/IMU.h>

using namespace std;

/*!
 * \brief For connecting to the Ardupilot IMU device.
 *
 * Connects over the /dev/ttyIMU serial port with 115200 baud rate.
 * \author Matthew Barulic
 * \headerfile Ardupilot.h <hardware/sensors/IMU/Ardupilot.h>
 */
class Ardupilot : public IMU
{
    public:
        Ardupilot();
        ~Ardupilot();
        Event<IMUData> onNewIMUData;

        bool isWorking();

    private:
        ASIOSerialPort ardupilotPort;

        void onNewSerialLine(string line);
        LISTENER(Ardupilot, onNewSerialLine, string);
};

#endif // ARDUPILOT_H
