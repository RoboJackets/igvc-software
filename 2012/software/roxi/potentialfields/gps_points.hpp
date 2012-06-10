#ifndef _GPS_POINTS_H_
#define _GPS_POINTS_H_

#define PRAC1
static const int num_GPS_Goals = 2;

#if defined(FourteenthStreet)
static const double goalWaypointLat[] = {33.787433,33.787256};
static const double goalWaypointLon[] = {-84.406306,-84.406356};
#elif defined(Competition2011)
static const double goalWaypointLat[] = {42.6781012505,42.67847883,42.6789,42.6793};
static const double goalWaypointLon[] = {-83.1948615905,-83.19472683,-83.1945,-83.1943};
#elif defined(QUALS)
static const double goalWaypointLat[] = {42.6779249,42.678193583,42.6779249};
static const double goalWaypointLon[] = {-83.195364772,-83.195414922,-83.195364772};
#elif defined(PRAC1)
static const double goalWaypointLat[] = {42.67828694, 42.67815921};
static const double goalWaypointLon[] = {-83.1949283, -83.19497341}; 
static const double goalRadius[] = {2, 2};
static const bool goalAvoidLines[] = {false, true};
#elif defined(PRAC2)
static const double goalWaypointLat[] = {42.67836202, 42.67848317};
static const double goalWaypointLon[] = {-83.19535227, -83.19506897};
static const double goalRadius[] = {2, 2};
static const bool goalAvoidLines[] = {false, true};
#else
static const double goalWaypointLat[] = {42.70};
static const double goalWaypointLon[] = {-83.194};
#endif

#endif
