#ifndef GPSDATA_H
#define GPSDATA_H

#include "SensorData.h"
#include "GPSAccuracy.hpp"


class GPSData : public SensorData
{
public:
    inline GPSData(double latitude, double longitude, double heading, double speed): SensorData(), _Lat(latitude), _Long(longitude),
      _Heading(heading), _Speed(speed), _Accuracy(NAV200Default)
    {
    }

    inline GPSData(double latitude, double longitude, double heading, double speed, double time): SensorData(time), _Lat(latitude),
     _Long(longitude), _Heading(heading), _Speed(speed), _Accuracy(NAV200Default)
    {
    }

    inline double Lat(void)
    {
        return _Lat;
    }

    inline double Long(void)
    {
        return _Long;
    }

    inline double Heading(void)
    {
        return _Heading;
    }

    inline double Speed(void)
    {
        return _Speed;
    }

    inline GPSAccuracy Accuracy(void)
    {
            return _Accuracy;
    }

    inline double LatVar(void)
    {
        return _Accuracy.LatVar();
    }

    inline double LongVar(void)
    {
        return _Accuracy.LongVar();
    }

    inline double HeadingVar(void)
    {
        return _Accuracy.HeadingVar();
    }

    inline double SpeedVar(void)
    {
        return _Accuracy.SpeedVar();
    }



    private:
        double _Lat;
        double _Long;
        double _Heading;
        double _Speed;
        GPSAccuracy _Accuracy;
        static GPSAccuracy NAV200Default; //TODO The default values are complete BS. Do some testing to find this stuff

};

//GPSAccuracy GPSData::NAV200Default = GPSAccuracy(.0001, .0001, 3, .3);


#endif // GPSDATA_H
