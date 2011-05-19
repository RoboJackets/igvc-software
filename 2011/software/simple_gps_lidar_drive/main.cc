
#include <iostream>

#include "gps.hpp"

#include "OSMC_4wd_driver.hpp"

#include "lidarProc.hpp"
#include "NAV200.hpp"

static const double waypointLat[] = {33};
static const double waypointLon[] = {8};

int main()
{
	NAV200 lidar;
	OSMC_4wd_driver motors;

	gps gpsA;
	gpsA.open("/dev/USB0", 38400);

	GPSState state;
	bool stateValid;
	stateValid = gpsA.get_last_state(state);
	while( (!stateValid) )
	{
		std::cout << "Waiting For Satilites" << std::endl;
		usleep(1e6);
		stateValid = gpsA.get_last_state(state);
	}

	//Average position for 10s
	usleep(10e6);
	
	//get new fix
	while( (!stateValid) )
	{
		std::cout << "Waiting For Satilites" << std::endl;
		usleep(2e5);
		stateValid = gpsA.get_last_state(state);
	}

	//get vector to first waypoint
	double x = waypointLat[0] - state.lon;
	double y = waypointLat[1] - state.lat;
	motors.set_vel_vec(y,x);
}
