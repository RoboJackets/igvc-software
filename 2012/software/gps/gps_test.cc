#include "gps.hpp"

#include <iostream>

#include <kdtree++/kdtree.hpp>

int main()
{
	gps gpsA;

	gpsA.open("/dev/ttyGPS", 4800);
	//gpsA.open("/dev/rfcomm0", 19200);
	gpsA.start();

	while(true)
	{
		GPSState state;
		if(gpsA.get_last_state(state))
		{
			//std::cout << "Qual: " << state.qual << " Sats: " << state.num_sat << " lat: " << state.lat << " lon: " << state.lon << std::endl;
			printf("qual: %i\tsat: %i\t%0.8f, %0.8f\n", state.qual, state.num_sat, state.lat, state.lon);
		}
		else
		{
			std::cout << "Error getting gps state" << std::endl;
		}
		usleep(1e6);
	}
}
