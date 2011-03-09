#include "NAV200.hpp"

#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <limits>

using namespace std;

bool NAV200::_usb_inited = false;

#ifndef LIDAR_SIMULATE
NAV200::NAV200()
{
	memset(valid, 0, Num_Points*sizeof(bool));
	memset(theta, 0, Num_Points*sizeof(float));
	memset(radius, 0, Num_Points*sizeof(float));
	//memset(x, 0, Num_Points*sizeof(float));
	//memset(y, 0, Num_Points*sizeof(float));
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
	//memset(x, 0, Num_Points*sizeof(float));
	//memset(y, 0, Num_Points*sizeof(float));
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

		valid[i] = pt.valid;
		if(pt.valid)
		{
			theta[i] = pt.angle;
			radius[i] = pt.distance;
			//polar2cart(pt.angle, pt.distance, x[i], y[i]);
		}
		else
		{
			radius[i] = theta[i] = std::numeric_limits<float>::quiet_NaN();
		}
	}

	return true;
}
#else
bool NAV200::read()
{
	return true;
}
#endif




