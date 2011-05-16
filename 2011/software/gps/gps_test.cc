#include "gps.hpp"

#include <iostream>

int main()
{
	gps gpsA;

	gpsA.open("/dev/ttyUSB0", 38400);
	gpsA.start();

	while(true)
	{
		gps::GPSState state = gpsA.get_last_pos();
		std::cout << "lat: " << state.lat << "\tlon: " << state.lon << std::endl;
		usleep(1e6);
	}

}
