#ifndef _GPS_POINTS_H_
#define _GPS_POINTS_H_

#define GPS_POINTS FourteenthStreet
static const int num_GPS_Goals = 4;
#if GPS_POINTS == FourteenthStreet
static const double goalWaypointLat[] = {33.787433};
static const double goalWaypointLon[] = {-84.406306};
#elif GPS_POINTS == Competition2011
static const double goalWaypointLat[] = {42.6781012505,42.67847883,42.6789,42.6793};
static const double goalWaypointLon[] = {-83.1948615905,-83.19472683,-83.1945,-83.1943};
#else
static const double goalWaypointLat[] = {42.70};
static const double goalWaypointLon[] = {-83.194};
#endif

#endif
