#ifndef GPS_COMMON
#define GPS_COMMON

#include <sys/time.h>

enum GPS_QUALITY {GPS_QUALITY_NOFIX=0, GPS_QUALITY_NON_DIFF=1, GPS_QUALITY_WAAS=2, GPS_QUALITY_ESTIMATED=6, GPS_QUALITY_UNKNOWN=7};


struct GPSState
{
	int num_sat;
	GPS_QUALITY qual;
	timeval laptoptime;
	double lat;
	double lon;
	double courseoverground;
};

//distance in meters
double lambert_distance(const GPSState& a, const GPSState& b);

#endif
