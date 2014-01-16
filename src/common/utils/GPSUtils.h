#ifndef GPSUTILS_H
#define GPSUTILS_H

#endif // GPSUTILS_H

#include <math.h>
#include <cmath>

class GPSUtils
{
    public:
        static void latLongCartesianUpdate(double oldLat, double oldLong, double deltaX, double deltaY, double &newLat, double &newLong)
        {
            newLat = oldLat + deltaY/111111;
            newLong = oldLong + deltaX/(111111*cos(oldLat));
        }

        static double coordsToMeter(double lat1, double lon1, double lat2, double lon2)
        {
            double dLat = (lat2 - lat1) * M_PI / 180;
            double dLon = (lon2 - lon1) * M_PI / 180;
            double a = sin(dLat/2) * sin(dLat/2) + cos(lat1 * M_PI / 180) * cos(lat2 * M_PI / 180) * sin(dLon/2) * sin(dLon/2);
            double c = 2 * atan2(sqrt(a), sqrt(1-a));
            double d = GPSUtils::EARTHRADIUS_KM * c;
            return d * 1000; // meters
        }

        static constexpr double EARTHRADIUS_KM = 6378.137;
};
