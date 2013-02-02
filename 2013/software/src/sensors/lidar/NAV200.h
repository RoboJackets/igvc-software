/*
 * NAV200.h
 *
 *  Created on: Jan 22, 2012
 *      Author: Alexander Huynh
Copypasta from: Matthew Barulic
 */
#ifndef NAV200_H
#define NAV200_H

#include "Lidar.h"

//#include <boost/thread.hpp>
#include "serial/ASIOSerialPort.h"



namespace IGVC {
namespace Sensors {

class NAV200: public IGVC::Sensors::Lidar {
public:
    NAV200();
    ~NAV200();
    LidarState GetState();
    LidarState GetStateAtTime(timeval time);
    bool StateIsAvailable();

private:

    ASIOSerialPort serialPort;
    boost::thread iothread;
    boost::mutex queueLocker;

    void threadRun(); // the method that runs on iothread
};

} /* namespace Sensors */
} /* namespace IGVC */
#endif // NAV200_H
