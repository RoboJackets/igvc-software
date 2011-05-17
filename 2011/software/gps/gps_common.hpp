#ifndef GPS_COMMON
#define GPS_COMMON

enum GPS_QUALITY {GPS_QUALITY_NOFIX=0, GPS_QUALITY_NON_DIFF=1, GPS_QUALITY_WAAS=2, GPS_QUALITY_ESTIMATED=6, GPS_QUALITY_UNKNOWN=7};


struct GPSState
{
	int numSat;
	GPS_QUALITY qual;
	double utc_time;
	double lat;
	double lon;
};

#endif
