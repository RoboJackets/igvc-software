#ifndef GPSDATA_H
#define GPSDATA_H

#include "SensorData.hpp"
#include <common/utils/gaussianvariable.hpp>

/*
 * An enumeration of GPS quality states.
 */
enum GPS_QUALITY {
	GPS_QUALITY_NOFIX=0,
	GPS_QUALITY_NON_DIFF=1,
	GPS_QUALITY_WAAS=2,
	GPS_QUALITY_ESTIMATED=6,
	GPS_QUALITY_UNKNOWN=7
};

class GPSData : public SensorData
{
public:

    inline GPSData()
        : SensorData()
    {
        _Lat = 0;
        _Long = 0;
        _Heading = 0;
        _Speed = 0;
    }

    // for use with gpsfilereader only
    inline GPSData(double latitude, double longitude): SensorData(), _Lat(latitude), _Long(longitude),
      _Heading(), _Speed()
    {
    }

    inline GPSData(double latitude, double longitude, double heading, double speed): SensorData(), _Lat(latitude), _Long(longitude),
      _Heading(heading), _Speed(speed)
    {
    }

    inline GPSData(double latitude, double longitude, double heading, double speed, double time): SensorData(time), _Lat(latitude),
     _Long(longitude), _Heading(heading), _Speed(speed)
    {
    }

    inline double Lat()
    {
        return _Lat;
    }

    inline double Long()
    {
        return _Long;
    }

    inline double Heading()
    {
        return _Heading;
    }

    inline double Speed()
    {
        return _Speed;
    }

    inline double LatVar()
    {
        return _Lat.Variance;
    }

    inline double LongVar()
    {
        return _Long.Variance;
    }

    inline double HeadingVar()
    {
        return _Heading.Variance;
    }

    inline double SpeedVar()
    {
        return _Speed.Variance;
    }

    inline GPS_QUALITY Quality()
    {
        return _quality;
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

    /*
    inline void  LatVar(double val)
    {
        _Lat.Variance = val;
    }

    inline void  LongVar(double val)
    {
        _Long.Variance = val;
    }

    inline void  HeadingVar(double val)
    {
        _Heading.Variance = val;
    }

    inline void  SpeedVar(double val)
    {
        _Speed.Variance = val;
    }
*/
    inline void Quality(GPS_QUALITY val)
    {
        _quality = val;
    }

    bool operator == (GPSData other)
    {
        return _Lat == other.Lat() &&
               _Long == other.Long() &&
               _Heading == other.Heading() &&
               _Speed == other.Speed();
    }

    friend std::ostream &operator<< (std::ostream &stream, GPSData &data)
    {
        stream << "(" << data.Lat() << ", " << data.Long() << ")";
        return stream;
    }

    private:
        GaussianVariable<double> _Lat;
        GaussianVariable<double> _Long;
        GaussianVariable<double> _Heading;
        GaussianVariable<double> _Speed;
        GPS_QUALITY _quality;

};

#endif // GPSDATA_H
