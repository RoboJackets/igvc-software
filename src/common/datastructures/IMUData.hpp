#ifndef IMUDATA_H
#define IMUDATA_H

#include "SensorData.hpp"
#include "GPSAccuracy.hpp"

class IMUData : public SensorData
{
public:
     // Defined according to NED convention
    IMUData(double roll=0, double pitch=0, double yaw=0, double x=0, double y=0, double z=0)
        : SensorData(), Roll(roll), Pitch(pitch), Yaw(yaw), X(x), Y(y), Z(z) { }

    // Units: Degrees
    double Roll;
    // Units: Degrees
    double Pitch;
    // Units: Degrees
    double Yaw;

    // Units: g's
    double X;
    // Units: g's
    double Y;
    // Units: g's
    double Z;
};

#endif // IMUDATA_H
