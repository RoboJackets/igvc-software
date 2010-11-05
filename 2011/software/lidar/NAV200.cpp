#include "NAV200.hpp"

#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <limits>

using namespace std;

bool tuple_sort_on_first(const boost::tuple<float,float>& a, const boost::tuple<float,float>& b)
{
	if(a.get<0>() < b.get<0>())
	{
		return true;
	}
	return false;
}

bool NAV200::_usb_inited = false;

#ifndef LIDAR_SIMULATE
NAV200::NAV200()
{
	memset(valid, 0, Num_Points*sizeof(bool));
	memset(theta, 0, Num_Points*sizeof(float));
	memset(radius, 0, Num_Points*sizeof(float));
	memset(x, 0, Num_Points*sizeof(float));
	memset(y, 0, Num_Points*sizeof(float));
	memset(points, 0, Num_Points*sizeof(Point));

	if (!_usb_inited)
	{
		usb_init();
		usb_find_busses();
		usb_find_devices();
	}
	
	_handle = 0;
	for (struct usb_bus *bus = usb_busses; bus; bus = bus->next)
	{
		for (struct usb_device *dev = bus->devices; dev; dev = dev->next)
		{
			if (dev->descriptor.idVendor == 0x3141 && dev->descriptor.idProduct == 0x0008)
			{
			    _handle = usb_open(dev);
				break;
			}
		}
	}
	
	if (!_handle)
	{
		throw runtime_error("Can't open USB device");
	}
	
	distance_scale = 0.8255 / 300;
}
#else
NAV200::NAV200()
{
	memset(valid, 0, Num_Points*sizeof(bool));
	memset(theta, 0, Num_Points*sizeof(float));
	memset(radius, 0, Num_Points*sizeof(float));
	memset(x, 0, Num_Points*sizeof(float));
	memset(y, 0, Num_Points*sizeof(float));
	memset(points, 0, Num_Points*sizeof(Point));

	std::cerr << "LIDAR simulate mode enabled!" << std::endl;
}
#endif

#ifndef LIDAR_SIMULATE
NAV200::~NAV200()
{
	usb_close(_handle);
}
#else
NAV200::~NAV200()
{

}
#endif

#ifndef LIDAR_SIMULATE
bool NAV200::read()
{
	uint8_t data[3083];
	memset(data, 0, sizeof(data));

	int ret = usb_control_msg(_handle, 0xc0, 4, 0, 0, (char *)data, sizeof(data), 500);
	if (ret != sizeof(data))
	{
		std::cerr << "Error in Read, ret = " << ret << std::endl;
		std::cerr << std::hex;
		for(int i = 0; i < ret; i++)
		{
			std::cerr << (int) data[i] << " ";
		}
		std::cerr << std::dec;
		// Command failed
		//FIXME - There are many possible errors here.
		return false;
	}
	
	uint8_t checksum = 0;
	for (int i = 0; i < (ret - 1); ++i)
	{
		checksum += data[i];
	}
	if (checksum != data[ret - 1])
	{
		// Bad checksum
		return false;
	}
	
	int start = data[5] + data[6] * 256;
	int offset = 7;
	for (int i = 0; i < Num_Points; ++i)
	{
		int ia = (i + start) % Num_Points;
		Point &pt = points[ia];
		
		pt.raw = data[offset] + data[offset + 1] * 256;
		pt.distance = pt.raw * distance_scale;
		pt.intensity = data[offset + 2];
		pt.valid = !(pt.raw & 0x4000);
		pt.angle = -ia * 2 * M_PI / NAV200::Num_Points;
		
		offset += 3;

		coord[i].get<0>() = pt.angle;
		coord[i].get<1>() = pt.distance;

		valid[i] = pt.valid;
		
		if(pt.valid)
		{
			theta[i] = pt.angle;
			radius[i] = pt.distance;
			polar2cart(theta[i], radius[i], x[i], y[i]);
		}
		else
		{
			x[i] = y[i] = radius[i] = theta[i] = std::numeric_limits<float>::quiet_NaN();
		}
	}
	
	std::sort(coord, coord + NAV200::Num_Points, tuple_sort_on_first);

	return true;
}
#else
bool NAV200::read()
{
	return true;
}
#endif

template<typename T>
void ref_abs(T& n)
{
	n = abs(n);
}

template<typename A, typename B>
void ref_abs_second_dim(boost::tuple<A,B>& n)
{
	n.get<1>() = abs(n.get<1>());
}

template<typename T>
void apply_func(std::list<T>& v, void (*f)(T&))
{
	BOOST_FOREACH(T& e, v)
	{
		f(e);
	}
}

template<typename T>
void apply_func(T* v, size_t len, void (*f)(T&))
{
	for(size_t i = 0; i < len; i++)
	{
		f(v[i]);
	}
}

//[a,b,c, ...]
//[b - a, c - b, ...]
template<typename A, typename B>
void NAV200::takeDerivative(const boost::tuple<A,B>* src, boost::tuple<A,B>* dest, const int srclen)
{
	for(int i = 0; i < (srclen-1); i++)
	{
		dest[i].get<0>() = src[i].get<0>() + (src[i+1].get<0>() - src[i].get<0>()) / (A(2));
		dest[i].get<1>() = (src[i+1].get<1>() - src[i].get<1>()) / (src[i+1].get<0>() - src[i].get<0>());
	}
}

bool NAV200::findLinearRuns(Point coord[Num_Points], std::deque< boost::tuple<size_t,size_t> >& lines, const double zero_tol)
{
	boost::tuple<float,float> coordtuple[Num_Points];

	for(int i = 0; i < Num_Points; i++)
	{
		coordtuple[i].get<0>() = coord[i].angle;
		coordtuple[i].get<1>() = coord[i].distance;
	}

	return findLinearRuns(coordtuple, lines, zero_tol);
}

bool NAV200::findLinearRuns(std::deque< boost::tuple<size_t,size_t> >& lines, const double zero_tol)
{
	return findLinearRuns(coord, lines, zero_tol);
}

bool NAV200::findLinearRuns(const boost::tuple<float,float> coord[Num_Points], std::deque< boost::tuple<size_t,size_t> >& lines, const double zero_tol)
{
	const int derivnum = Num_Points-1;
	const int doublederivnum = Num_Points-2;

	boost::tuple<float,float> distance_deriv[derivnum];
	boost::tuple<float,float> distance_second_deriv[doublederivnum];
	boost::tuple<float,float> abs_distance_second_deriv[doublederivnum];

	takeDerivative(coord, distance_deriv, Num_Points);
	takeDerivative(distance_deriv, distance_second_deriv, derivnum);

	memcpy(abs_distance_second_deriv, distance_second_deriv, sizeof(distance_second_deriv));
	apply_func(abs_distance_second_deriv, doublederivnum, ref_abs_second_dim);

	bool linear_map[doublederivnum] = {false};
	int start, stop;
	start = stop = -1;
	for(int i = 0; i < doublederivnum; i++)
	{

		if(abs_distance_second_deriv[i].get<1>() < zero_tol)
		{
			linear_map[i] = true;
		}

		//start a range
		if( (linear_map[i]) && (start == -1))
		{
			start = i;
		}

		//stop a range
		if( ( !(linear_map[i]) ) && (start != -1) && (i != (doublederivnum-1)) )//we are not at last element, have an open range. set the end to the next element
		{
			stop = i+1;
			lines.push_back( boost::tuple<size_t,size_t>(start, stop) );
			start = stop = -1;
		}
		else if( (( !(linear_map[i]) ) || (i == (doublederivnum-1))) && (start != -1) )//we are at last element, and still have an open range. set the end to the last element
		{
			stop = i+2;

			lines.push_back( boost::tuple<size_t,size_t>(start, stop) );
			start = stop = -1;
		}
	}

	return true;
}

void NAV200::getLongestRun(Point coord[Num_Points], const std::deque< boost::tuple<size_t,size_t> >& lines, boost::tuple<size_t,size_t>& longest)
{
	//fix this to use the lines to index into coord and calculate the length of a line segment. remeber to transform out of polar coord
	float maxdist = -1;
	size_t idx = -1;

	for(size_t i = 0; i < lines.size(); i++)
	{
		float len = lines[i].get<1>() - lines[i].get<0>();

		if(maxdist == len)
		{
			float old_dist_straight = abs(M_PI / double(2) - (lines[idx].get<1>() - lines[idx].get<0>()) / (float(2)) );
			float new_dist_straight = abs(M_PI / double(2) - (lines[i].get<1>() - lines[i].get<0>()) / (float(2)) );

			if(old_dist_straight > new_dist_straight)
			{
				maxdist = len;
				idx = i;
			}
			continue;
		}

		if(maxdist < len)
		{
			maxdist = len;
			idx = i;
		}
	}

	longest = lines[idx];
}
