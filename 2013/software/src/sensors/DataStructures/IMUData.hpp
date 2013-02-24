#ifndef IMUDATA_H
#define IMUDATA_H

#include "SensorData.h"
#include "GPSAccuracy.hpp"

class IMUData : public SensorData
{
public:
    inline IMUData(double deltaX, double deltaY, double Heading, double Speed, double deltaTime) : SensorData(), _deltaX(deltaX),
     _deltaY(deltaY), _Heading(Heading),_deltaTime(deltaTime), _Accuracy(ArduPilotDefault)
     {
     }

    inline double deltaX(void)
    {
        return _deltaX;
    }

    inline double deltaY(void)
    {
        return _deltaY;
    }

    inline double Heading(void)
    {
        return _Heading;
    }

    inline double Speed(void)
    {
        return _Speed;
    }

    inline double deltaTime(void)
    {
        return _deltaTime;
    }

private:
    double _deltaX; //East being positive
    double _deltaY; //North being positive
    double _Heading;
    double _Speed;
    double _deltaTime;
    GPSAccuracy _Accuracy;
    static GPSAccuracy ArduPilotDefault;
};

//GPSAccuracy IMUData::ArduPilotDefault= GPSAccuracy(.001, .001, 1, .1);

#endif // IMUDATA_H
