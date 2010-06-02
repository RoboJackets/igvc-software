#pragma once

#include <usb.h>

#include <list>
#include <deque>

#include "boost/tuple/tuple.hpp"
#include "boost/foreach.hpp"

//if defined, will not connect to NAV200
#define LIDAR_SIMULATE

// NAV200, communication interace for the lidar
// original code by ben
// integration by jacob

class NAV200
{
	public:
		class Point
		{
			public:
				Point()
				{
					valid = false;
					distance = 0;
					intensity = 0;
				}
				
				// If false, the other members are not meaningful
				bool valid;
				
				// Angle in radians counterclockwise from right, with the LEDs pointing forward.
				float angle;
				
				// Raw distance
				uint16_t raw;
				
				// Distance in meters
				float distance;
				
				// Intensity of return, unknown units
				uint8_t intensity;
		};
		
		static const int Num_Points = 1024;
		
		// FIXME - Identify one of many devices
		NAV200();
		~NAV200();
		
		bool read();
		
		// Multiplied by raw values to get distance.
		float distance_scale;
		
		Point points[Num_Points];
		boost::tuple<float,float> coord[Num_Points];

		//do some proccessing
		bool findLinearRuns(std::deque< boost::tuple<float,float> >& lines);
		static bool findLinearRuns(const boost::tuple<float,float> coord[Num_Points], std::deque< boost::tuple<float,float> >& lines);

		static void getLongestRun(const std::deque< boost::tuple<float,float> >& lines, boost::tuple<float,float>& longest);

	private:
		//FIXME - This should be in libusb.  Find it.
		static bool _usb_inited;
	
	#ifndef LIDAR_SIMULATE	
		usb_dev_handle *_handle;
	#endif
};
