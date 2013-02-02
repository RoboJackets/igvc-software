#ifndef GPSDATA_H
#define GPSDATA_H

#include <SensorData.h>

class GPSData : public SensorData
{
public:
    inline GPSData(double latitude, double longitude, double heading, double speed): SensorData(), theLat(latitude), theLong(longitude), theHeading(heading), theSpeed(speed)
    {
    }

    inline double Lat(void)
    {
        return theLat;
    }

    inline double Long(void)
    {
        return theLong;
    }

    inline double Heading(void)
    {
        return theHeading;
    }

    inline double Speed(void)
    {
        return theSpeed;
    }

    private:
        double theLat;
        double theLong;
        double theHeading;
        double theSpeed;
};

#endif // GPSDATA_H
