#ifndef ARDUPILOT_H
#define ARDUPILOT_H

#include <hardware/serial/SerialPort.h>
#include <common/datastructures/IMUData.hpp>
#include <hardware/sensors/IMU/IMU.h>
#include <QObject>

/*!
 * \brief For connecting to the Ardupilot IMU device.
 *
 * Connects over the /dev/ttyIMU serial port with 115200 baud rate.
 * \author Matthew Barulic
 * \headerfile Ardupilot.h <hardware/sensors/IMU/Ardupilot.h>
 */
class Ardupilot : public IMU
{
    Q_OBJECT
    public:
        Ardupilot();
        ~Ardupilot();

        bool isWorking();

    private slots:
        void onNewSerialLine(std::string line);

    private:
        SerialPort ardupilotPort;
};

#endif // ARDUPILOT_H
