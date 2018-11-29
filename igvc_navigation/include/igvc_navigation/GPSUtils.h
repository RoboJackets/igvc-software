#ifndef GPSUTILS_H
#define GPSUTILS_H

#include <math.h>
#include <cmath>

namespace GPSUtils
{
constexpr double EARTHRADIUS_KM = 6378.137;

double coordsToMeter(double lat1, double lon1, double lat2, double lon2)
{
  double dLat = (lat2 - lat1) * (M_PI / 180.0);
  double dLon = (lon2 - lon1) * (M_PI / 180.0);
  lat1 = lat1 * (M_PI / 180.0);
  lat2 = lat2 * (M_PI / 180.0);
  double a = sin(dLat / 2.0) * sin(dLat / 2.0) + cos(lat1) * cos(lat2) * sin(dLon / 2.0) * sin(dLon / 2.0);
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  double d = GPSUtils::EARTHRADIUS_KM * c;
  return d * 1000.0;  // meters
}

void coordAfterMotion(double lat1, double lon1, double d, double a, double &newLat, double &newLon)
{
  // angle in radians
  // dist in meters
  d = d / (EARTHRADIUS_KM * 1000.0);  // angle corresponding to arc length/dist on earth
  // tc = tc * (M_PI/180.0); //angle from initial point to current location
  lat1 = lat1 * (M_PI / 180.0);
  lon1 = lon1 * (M_PI / 180.0);
  newLat = asin(sin(lat1) * cos(d) + cos(lat1) * sin(d) * cos(a));
  if (cos(newLat) == 0)
    newLon = lon1;  // endpoint a pole
  else
    newLon = fmod(lon1 - asin(sin(a) * sin(d) / cos(newLat)) + M_PI, 2.0 * M_PI) - M_PI;
  newLat = newLat * (180.0 / M_PI);
  newLon = newLon * (180.0 / M_PI);
}

/**
 * @brief Takes two GPS coordinates and outputs X and Y displacements.
 */
void coordsToMetricXY(double lat1, double lon1, double lat2, double lon2, double &dX, double &dY)
{
  dX = std::copysign(1.0, lon2 - lon1) * coordsToMeter(lat1, lon1, lat1, lon2);
  dY = std::copysign(1.0, lat2 - lat1) * coordsToMeter(lat1, lon1, lat2, lon1);
}
};  // namespace GPSUtils

#endif