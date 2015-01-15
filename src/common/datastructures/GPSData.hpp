#ifndef GPSDATA_H
#define GPSDATA_H

#include "SensorData.hpp"
#include <ostream>
#include <common/utils/gaussianvariable.hpp>
#include <QTextStream>
#include <iomanip>

/*
 * An enumeration of GPS quality states.
 */
enum GPS_QUALITY {
    GPS_QUALITY_INVALID=0,
    GPS_QUALITY_SPS=1,
    GPS_QUALITY_DGPS=2,
    GPS_QUALITY_PPS=3,
    GPS_QUALITY_RTK=4,
    GPS_QUALITY_Float_RTK=5,
    GPS_QUALITY_Estimated=6,
    GPS_QULAITY_Manual=7,
    GPS_QUALITY_Simulation=8
};

class GPSData : public SensorData
{
public:

    inline GPSData(double latitude=0, double longitude=0, double heading=0, double speed=0, double time=0, int numSats=0, GPS_QUALITY quality=GPS_QUALITY_INVALID, float hdop=0)
        : SensorData(time),
          _Lat(latitude),
          _Long(longitude),
          _Heading(heading),
          _Speed(speed),
          _quality(quality),
          _NumSats(numSats),
          _HDOP(hdop)
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

    inline int NumSats()
    {
        return _NumSats;
    }

    inline float HDOP()
    {
        return _HDOP;
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

    inline void Quality(GPS_QUALITY val)
    {
        _quality = val;
    }

    inline void NumSats(int val)
    {
        _NumSats = val;
    }

    inline void HDOP(float val)
    {
        _HDOP = val;
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
        stream << std::setprecision(15) << data.Lat() << ',' << data.Long() << ',' << data.NumSats() << ',' << (int)data.Quality()  << ',' << data.HDOP();
        return stream;
    }

    friend QTextStream &operator<< (QTextStream &stream, GPSData &data) {
        stream << qSetRealNumberPrecision(15) << data.Lat() << ',' << data.Long() << ',' << data.NumSats() << ',' << (int)data.Quality()  << ',' << data.HDOP();
        return stream;
    }

    private:
        GaussianVariable<double> _Lat;
        GaussianVariable<double> _Long;
        GaussianVariable<double> _Heading;
        GaussianVariable<double> _Speed;
        GPS_QUALITY _quality;
        int _NumSats;
        float _HDOP; // http://en.wikipedia.org/wiki/Dilution_of_precision_(GPS)

};

#endif // GPSDATA_H
