#ifndef IMU_H
#define IMU_H

#include <common/module.hpp>
#include <common/datastructures/IMUData.hpp>

/*!
 * \brief Interface for IMU devices.
 * \headerfile IMU.h <hardware/sensors/IMU/IMU.h>
 */
class IMU : public Module
{
    Q_OBJECT
signals:
    void onNewData(IMUData);
public:
    IMU() {
        qRegisterMetaType<IMUData>("IMUData");
    }
};



#endif //IMU_H
