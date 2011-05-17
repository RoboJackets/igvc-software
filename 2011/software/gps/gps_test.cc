#include "gps.hpp"

#include <iostream>

int main()
{
	gps gpsA;

	//gpsA.open("/dev/ttyUSB0", 38400);
	gpsA.open("/dev/rfcomm0", 19200);
	gpsA.start();

	while(true)
	{
		GPSState state;
		if(gpsA.get_last_pos(state))
		{
			std::cout << "Qual: " << state.qual << " Sats: " << state.num_sat << " lat: " << state.lat << " lon: " << state.lon << std::endl;
		}
		else
		{
			std::cout << "Error getting gps state" << std::endl;
		}
		usleep(1e6);
	}

}
