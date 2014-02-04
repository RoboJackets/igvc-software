#ifndef GPSDATA_H
#define GPSDATA_H

#include "SensorData.hpp"
//#include <sensors/GPS/GPS.hpp>
#include "GPSAccuracy.hpp"

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
        : SensorData(),
        _Accuracy()
    {
        _Lat = 0;
        _Long = 0;
        _Heading = 0;
        _Speed = 0;
    }

    // for use with gpsfilereader only
    inline GPSData(double latitude, double longitude): SensorData(), _Lat(latitude), _Long(longitude),
      _Heading(), _Speed(), _Accuracy()
    {
    }

    inline GPSData(double latitude, double longitude, double heading, double speed): SensorData(), _Lat(latitude), _Long(longitude),
      _Heading(heading), _Speed(speed), _Accuracy()
    {
    }

    inline GPSData(double latitude, double longitude, double heading, double speed, double time): SensorData(time), _Lat(latitude),
     _Long(longitude), _Heading(heading), _Speed(speed), _Accuracy()
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

    inline GPS_QUALITY Quality(void)
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

    inline void  Accuracy(GPSAccuracy val)
    {
            _Accuracy = val;
    }

    /*
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
//                     _Accuracy == other.Accuracy() &&

    }

    friend std::ostream &operator<< (std::ostream &stream, GPSData &data)
    {
        stream << "(" << data.Lat() << ", " << data.Long() << ")";
        return stream;
    }

    private:
        double _Lat;
        double _Long;
        double _Heading;
        double _Speed;
        GPSAccuracy _Accuracy;
        GPS_QUALITY _quality;

};

//GPSAccuracy GPSData::NAV200Default = GPSAccuracy(.0001, .0001, 3, .3);


#endif // GPSDATA_H
