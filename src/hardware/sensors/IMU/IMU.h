#ifndef IMU_H
#define IMU_H

#include <common/events/Event.hpp>
#include <common/datastructures/IMUData.hpp>

/*!
 * \brief Interface for IMU devices.
 * \headerfile IMU.h <hardware/sensors/IMU/IMU.h>
 */
class IMU
{
    public:
        IMU() { }
        Event<IMUData>onNewData;

        /*! \brief Returns true if the device is working correctly. */
        virtual bool isWorking() = 0;
};



#endif //IMU_H
