#ifndef IMUDATA_H
#define IMUDATA_H

#include "SensorData.h"
#include "GPSAccuracy.hpp"

class IMUData : public SensorData
{
public:
    inline IMUData()
    {
    }

    /**
    Defined according to NED convention
    **/
    inline IMUData(double roll, double pitch, double yaw) : SensorData(), _Roll(roll), _Pitch(pitch), _Yaw(yaw)
    {
    }

    /**
    Units: Radians
    **/
    inline double Roll(void)
    {
        return _Roll;
    }

    /**
    Units: Radians
    **/
    inline double Pitch(void)
    {
        return _Pitch;
    }

    /**
    Units: Radians
    **/
    inline double Yaw(void)
    {
        return _Yaw;
    }

private:
    double _Roll;
    double _Pitch;
    double _Yaw;
};

#endif // IMUDATA_H
