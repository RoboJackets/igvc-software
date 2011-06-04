
#include <iostream>

#include "gps.hpp"

#include "OSMC_4wd_driver.hpp"

#include "lidarProc.hpp"
#include "NAV200.hpp"

/*//To run the vision code with the gps
#include "main.h"
#include "Robot.h"
#include <QApplication>*/

//static const double waypointLat[] = {33.787175,  33.787196};
//static const double waypointLon[] = {-84.406264, -84.406066};
//static const size_t numPts = 1;


//static const double waypointLat[] = {33.787195};
//static const double waypointLon[] = {-84.406886};
//static const size_t numPts = 1;

static const double waypointLat[] = {42.67888880473297};
static const double waypointLon[] = {-83.19609817733424};
static const size_t numPts = 1;

int main()
{
	NAV200 lidar;
	OSMC_4wd_driver motors;

	motors.setLight(MC_LIGHT_PULSING);

	gps gpsA;
	gpsA.open("/dev/ttyUSB2", 38400);
	//gpsA.open("/dev/rfcomm0", 19200);

	gpsA.start();

	GPSState state;
	{
	bool stateValid = gpsA.get_last_state(state);
	while( (!stateValid) )
	{
		std::cout << "Waiting For Satellites" << std::endl;
		usleep(1e5);
		stateValid = gpsA.get_last_state(state);
	}
	}

	std::cout << "Avg 10s" << std::endl;
	//Average position for 10s
	usleep(10e6);
	
	//get new fix
	{
	bool stateValid = gpsA.get_last_state(state);
	while( (!stateValid) )
	{
		std::cout << "Waiting For Satellites" << std::endl;
		usleep(1e5);
		stateValid = gpsA.get_last_state(state);
	}
	}

	float goodtheta[NAV200::Num_Points];
	float goodradius[NAV200::Num_Points];
	float runavg_goodradius[NAV200::Num_Points];
	for(size_t i = 0; i < numPts; i++)
	{
		GPSState target;
		target.lat = waypointLat[i];
		target.lon = waypointLon[i];
		double distance = lambert_distance(state, target);

		while( distance > 2.0 )
		{
			{
			bool stateValid = gpsA.get_last_state(state);
			while( (!stateValid) )
			{
				std::cout << "Waiting For Satellites" << std::endl;
				usleep(2e5);
				stateValid = gpsA.get_last_state(state);
			}
			}
			printf("%0.8f, %0.8f\t//\t%0.8f, %0.8f\n", state.lat, state.lon, waypointLat[i], waypointLon[i]);
			//std::cout << state.lat << ", " << state.lon << "\t//\t" << waypointLat[i]<< ", " << waypointLon[i] << std::endl;

			distance = lambert_distance(state, target);
			std::cout << "Distance to go: " << distance << " m" << std::endl;
			//get lidar data
			if(!lidar.read())
			{
				std::cerr << "No LIDAR Data, using old data" << std::endl;
			}
			//copy the good points
			size_t numlidarpts = lidar.getValidData(goodtheta, goodradius);
			//running average
			lidarProc::runavg(goodradius, runavg_goodradius, numlidarpts, 30);

			//get vector to first waypoint
			double x = waypointLon[i] - state.lon;
			double y = waypointLat[i] - state.lat;
			double targetvector = atan2(y,x);

			//local 
			double angle_to_target = targetvector - (M_PI_2 - state.courseoverground);

			std::cout << "Angle to go (pre lidar): " << angle_to_target << " rad" << std::endl;

			//can we go the dir we want?
			bool clear = lidarProc::isPathClear(angle_to_target, 1, 1, goodtheta, runavg_goodradius, numlidarpts);

			//if not, find the closest angle
			while(!clear)
			{
				if(angle_to_target >= 0)
				{
					angle_to_target += .05;
					clear = lidarProc::isPathClear(angle_to_target, 1, 1, goodtheta, runavg_goodradius, numlidarpts);
				}
				else
				{
					angle_to_target -= .05;
					clear = lidarProc::isPathClear(angle_to_target, 1, 1, goodtheta, runavg_goodradius, numlidarpts);
				}

				//go straight if robot can't find a path
				if(fabsf(angle_to_target) > (2.0*M_PI))
				{
					clear = true;
					angle_to_target = M_PI_2;
				}
			}
			std::cout << "Angle to go (post lidar): " << angle_to_target << " rad" << std::endl;
			//got the angle we decided
			motors.set_vel_vec(sin(angle_to_target),cos(angle_to_target));
			usleep(1e3);
		}
		distance = lambert_distance(state, target);
		std::cout << "Waypoint " << i << " hit at final distance " << distance << " m" << std::endl;
	}

	motors.setLight(MC_LIGHT_STEADY);
}
