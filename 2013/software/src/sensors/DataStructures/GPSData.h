#ifndef GPSDATA_H
#define GPSDATA_H

#include "SensorData.h"
#include "GPSAccuracy.hpp"


class GPSData : public SensorData
{
public:

    inline GPSData()
        : SensorData(),
        _Accuracy(NAV200Default)
    {
        _Lat = 0;
        _Long = 0;
        _Heading = 0;
        _Speed = 0;
    }

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

    inline void Lat(double val)
    {
        _Lat = val;
    }

    inline void  Long(double val)
    {
        _Long = val;
    }

    inline void  Heading(double val)
    {
        _Heading = val;
    }

    inline void  Speed(double val)
    {
        _Speed = val;
    }

    inline void  Accuracy(GPSAccuracy val)
    {
            _Accuracy = val;
    }

    inline void  LatVar(double val)
    {
        //TODO set variance
    }

    inline void  LongVar(double val)
    {
        //TODO set variance
    }

    inline void  HeadingVar(double val)
    {
        //TODO set variance
    }

    inline void  SpeedVar(double val)
    {
        //TODO set variance
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
