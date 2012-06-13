#ifndef _GPS_POINTS_H_
#define _GPS_POINTS_H_

#define COMPCW

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
static const double goalRadius[] = {1, 1};
static const bool goalIgnoreLines[] = {true, false};
#elif defined(PRAC2)
static const double goalWaypointLat[] = {42.67836202, 42.67848317};
static const double goalWaypointLon[] = {-83.19535227, -83.19506897};
static const double goalRadius[] = {1, 1};
static const bool goalIgnoreLines[] = {false, false};
#elif defined(COMPCW)
static const int num_GPS_Goals = 11;
										// AUVSI		// TARDEK		// OAKLAND		// SIAC			// GatePt1		// GatePt2		// LANE			// SOD			// CHEOK		// TACOM		//retcornerS
static const double goalWaypointLat[] = {42.67955517, 	42.67948318,   	42.67922799, 	42.67927407, 	42.67927407,	42.67901806,	42.67901806,	42.6788151, 	42.67907928, 	42.67875409,		42.6787};
static const double goalWaypointLon[] = {-83.19487315, 	-83.19518328,  	-83.19499775, 	-83.19534374,	-83.19517282,	-83.19517282, 	-83.1953296, 	-83.19514147,	-83.19498461,  	-83.19483987,	-83.1956};
static const double goalRadius[] = 		{1, 			2, 				2, 				1, 				1,				1,				1, 				2, 				2, 				1				20};
static const bool goalIgnoreLines[] = 	{false,			true,			true,			true,			false,			false,			true,			true,			true, 			false			false};
#elif defined(COMPCCW)
static const int num_GPS_Goals = 11;
										// TACOM		// CHEOK		// SOD			// LANE			// GatePt1		// GatePt2		// SIAC			// OAKLAND		// TARDEK		// AUVSI	//retcornerN
static const double goalWaypointLat[] = {42.67875409, 	42.67907928,	42.6788151,     42.67901806, 	42.67901806,	42.67927407		42.67927407, 	42.67922799,	42.67948318, 	42.67955517,		42.67955};
static const double goalWaypointLon[] = {-83.19483987, 	-83.19498461,	-83.19514147,	-83.1953296,  	-83.19517282, 	-83.19517282,	-83.19534374, 	-83.19499775,	-83.19518328,	-83.19487315,	-83.1956};
static const double goalRadius[] = 		{1, 			1,				2, 				1, 				1,				1,				1,				2, 				1, 				1				20};
static const bool goalIgnoreLines[] = 	{false,			true,			true,			true,			false,			false			true,			true,			true,			false			false};
#else
static const double goalWaypointLat[] = {42.70};
static const double goalWaypointLon[] = {-83.194};
#endif

#endif
/*
42.6791667
83.1954667
*/
//longperfence=.0000 3135 5
/*
.1956//st
.1949//ed
range=.0007
*/
