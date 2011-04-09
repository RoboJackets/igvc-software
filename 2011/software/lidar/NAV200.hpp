#pragma once

#include <cmath>
#include <limits>
#include <list>
#include <deque>

#include <usb.h>

#include "boost/tuple/tuple.hpp"
#include "boost/foreach.hpp"

//if defined, will not connect to NAV200
//#define LIDAR_SIMULATE

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
		
		//normal
		Point points[Num_Points];

		//coord + mask
		bool valid[Num_Points];
		float theta[Num_Points];
		float radius[Num_Points];
		//float x[Num_Points];
		//float y[Num_Points];

		static void polar2cart(const float angle, const float radius, float& x, float& y)
		{
			float s,c;
			sincosf(angle, &s, &c);
			x = radius * c;
			y = radius * s;
		}

		static void polar2cart(const float* angle, const float* radius, float* x, float* y, const size_t len)
		{
			for(size_t i = 0; i < len; i++)
			{
				polar2cart(angle[i], radius[i], x[i], y[i]);
			}
		}

		static bool polar2cart(const Point& pt, float& x, float& y)
		{
			if(pt.valid)
			{
				polar2cart(pt.angle, pt.distance, x, y);
				return true;
			}
			else
			{
				x = std::numeric_limits<float>::signaling_NaN();
				y = std::numeric_limits<float>::signaling_NaN();
			}
			return false;
		}

		static void cart2polar(const float x, const float y, float& angle, float& radius)
		{
			angle = atan2(y,x);
			radius = sqrt(x*x + y*y);
		}

		static void cart2polar(const float* x, const float* y, float* angle, float* radius, const size_t len)
		{
			for(size_t i = 0; i < len; i++)
			{
				cart2polar(x[i], y[i], angle[i], radius[i]);
			}
		}

		size_t getValidData(float thetaout[Num_Points], float radiusout[Num_Points])
		{
			size_t goodpts = 0;
			for(int i = 0; i < Num_Points; i++)
			{
				if(valid[i])
				{
					thetaout[goodpts] = theta[i];
					radiusout[goodpts] = radius[i];
					goodpts++;
				}
			}
			return goodpts;
		}

	private:
		//FIXME - This should be in libusb.  Find it.
		static bool _usb_inited;
	
	#ifndef LIDAR_SIMULATE	
		usb_dev_handle *_handle;
	#endif
};
