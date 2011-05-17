#include "gps.hpp"

#include <iostream>

int main()
{
	gps gpsA;

	gpsA.open("/dev/ttyUSB0", 38400);
	gpsA.start();

	while(true)
	{
		GPSState state;
		if(gpsA.get_last_pos(state))
		{
			std::cout << "Qual: " << state.qual << " lat: " << state.lat << "\tlon: " << state.lon << std::endl;
		}
		else
		{
			std::cout << "Error in packet" << std::endl;
		}
		usleep(1e6);
	}

}
