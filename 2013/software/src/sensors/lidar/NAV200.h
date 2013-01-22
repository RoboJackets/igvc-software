#ifndef NAV200_H
#define NAV200_H

#include "Lidar.h"

#include <boost/thread.hpp>
#include "../../serial/ASIOSerialPort.h"

namespace IGVC {
namespace Sensors {

class NAV200
{
    public:
        NAV200();
        virtual ~NAV200();
    private:
        //ASIOSerialPort serialPort;
        //boost::thread iothread;
        //boost::mutex queueLocker;

        void threadRun(); // the method that runs on iothread
};

} /* namespace Sensors */
} /* namespace IGVC */
#endif // NAV200_H
