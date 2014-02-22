#ifndef IMU_H
#define IMU_H

#include <QObject>
#include <common/datastructures/IMUData.hpp>

/*!
 * \brief Interface for IMU devices.
 * \headerfile IMU.h <hardware/sensors/IMU/IMU.h>
 */
class IMU : public QObject
{
    Q_OBJECT
signals:
    void onNewData(IMUData);
public:
    IMU() { }

    /*! \brief Returns true if the device is working correctly. */
    virtual bool isWorking() = 0;
};



#endif //IMU_H
