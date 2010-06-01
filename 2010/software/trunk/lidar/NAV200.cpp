#include "NAV200.hpp"

#include <stdexcept>
#include <cmath>

using namespace std;

bool NAV200::_usb_inited = false;

NAV200::NAV200()
{
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

NAV200::~NAV200()
{
	usb_close(_handle);
}

bool NAV200::read()
{
	uint8_t data[3083];
	int ret = usb_control_msg(_handle, 0xc0, 4, 0, 0, (char *)data, sizeof(data), 500);
	if (ret != sizeof(data))
	{
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
	}
	
	return true;
}

template<typename T>
void ref_abs(T& n)
{
	n = abs(n);
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
template<typename T>
void takeDerivative(const T* src, T* dest, const int srclen)
{
	for(int i = 0; i < (srclen-1); i++)
	{
		dest[i] = src[i+1] - src[i];
	}
}

bool NAV200::findLinearRuns(std::list< boost::tuple<int,int> >& lines)
{
	if(!read())
	{
		return false;
	}

	return findLinearRuns(points, lines);
}

bool NAV200::findLinearRuns(const Point pts[Num_Points], std::list< boost::tuple<int,int> >& lines)
{
	//todo: load distance
	float distance[Num_Points];
	for(int i = 0; i < Num_Points; i++)
	{
		distance[i] = pts[i].distance;
	}

	return findLinearRuns(distance, lines);
}

bool NAV200::findLinearRuns(const float distance[Num_Points], std::list< boost::tuple<int,int> >& lines)
{
	const double zero_tol = 1e-3;



	const int derivnum = Num_Points-1;
	const int doublederivnum = Num_Points-2;

	float distance_deriv[derivnum];
	float distance_second_deriv[doublederivnum];
	float abs_distance_second_deriv[doublederivnum];

	takeDerivative(distance, distance_deriv, Num_Points);
	takeDerivative(distance_deriv, distance_second_deriv, derivnum);

	memcpy(abs_distance_second_deriv, distance_second_deriv, doublederivnum*sizeof(float));
	apply_func(abs_distance_second_deriv, doublederivnum, ref_abs);


	bool linear_map[doublederivnum] = {false};
	for(int i = 0; i < doublederivnum; i++)
	{
		if(abs_distance_second_deriv[i] < zero_tol)
		{
			linear_map[i] = true;
		}
	}
	
	int start, stop;
	start = stop = -1;
	for(int i = 0; i < doublederivnum; i++)
	{
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
