/*
 * NAV200.h
 *
 *  Created on: Jan 22, 2012
 *      Author: Alexander Huynh
 */
#ifndef NAV200_H
#define NAV200_H

#include "Lidar.h"
#include <libusb-1.0/libusb.h>
#include <boost/thread.hpp>

namespace IGVC {
namespace Sensors {

class NAV200: public Lidar {
public:
    NAV200();
    ~NAV200();
    LidarState GetState();
    LidarState GetStateAtTime(timeval time);
    bool IsWorking();

private:
    boost::thread _iothread;
    boost::mutex _queueLocker;
    libusb_device_handle *_handle;
    libusb_context *_ctx;
    bool _running;
    int _numPoints;

    void threadRun(); // the method that runs on iothread
};

} /* namespace Sensors */
} /* namespace IGVC */
#endif // NAV200_H
