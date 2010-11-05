#pragma once

#include <cmath>
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

		//dumb - to be deprecated
		boost::tuple<float,float> coord[Num_Points];

		//polar coord + mask
		bool valid[Num_Points];
		float theta[Num_Points];
		float radius[Num_Points];
		float x[Num_Points];
		float y[Num_Points];

		//do some proccessing
		bool findLinearRuns(std::deque< boost::tuple<size_t,size_t> >& lines, const double zero_tol);
		static bool findLinearRuns(const boost::tuple<float,float> coord[Num_Points], std::deque< boost::tuple<size_t,size_t> >& lines, const double zero_tol);
		static bool findLinearRuns(Point coord[Num_Points], std::deque< boost::tuple<size_t,size_t> >& lines, const double zero_tol);

		static void getLongestRun(Point coord[Num_Points], const std::deque< boost::tuple<size_t,size_t> >& lines, boost::tuple<size_t,size_t>& longest);

		static void polar2cart(const float angle, const float radius, float& x, float& y)
		{
			float s,c;
			sincosf(angle, &s, &c);
			x = radius * c;
			y = radius * s;
		}

		static void polar2cart(const boost::tuple<float, float>& pt, float& x, float& y)
		{
			polar2cart(pt.get<0>(), pt.get<1>(), x, y);
		}

		static bool polar2cart(const Point& pt, float& x, float& y)
		{
			if(pt.valid)
			{
				polar2cart(pt.angle, pt.distance, x, y);
				return true;
			}
			return false;
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

	//[a,b,c, ...]
	//[b - a, c - b, ...]
	template<typename A, typename B>
	static void takeDerivative(const boost::tuple<A,B>* src, boost::tuple<A,B>* dest, const int srclen);

	//[a,b,c, ...]
	//[b - a, c - b, ...]
	template<typename T>
	static void takeDerivative(const T* srcX, const T* srcY, T* destX, T* destY, const int srclen)
	{
		for(int i = 0; i < (srclen-1); i++)
		{
			destX[i] = srcX[i] + (srcX[i+1] - srcX[i]) / (T(2));
			destY[i] = (srcY[i+1] - srcY[i]) / (srcX[i+1] - srcX[i]);
		}
	}

	private:
		//FIXME - This should be in libusb.  Find it.
		static bool _usb_inited;
	
	#ifndef LIDAR_SIMULATE	
		usb_dev_handle *_handle;
	#endif
};
