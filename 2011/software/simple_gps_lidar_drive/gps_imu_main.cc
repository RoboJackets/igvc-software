#include "gyro.hpp"

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

//static const double waypointLat[] = {42.67888880473297};
//static const double waypointLon[] = {-83.19609817733424};
//static const size_t numPts = 1;

static const double waypointLat[] = {42.6781012505};
static const double waypointLon[] = {-83.1948615905};
static const size_t numPts = 1;

float convertyaw(float olddeg)
{
	return -olddeg*M_PI/180.0;
}

int main()
{
	NAV200 lidar;
	OSMC_4wd_driver motors;

	motors.setLight(MC_LIGHT_PULSING);

	gyro gyroA;
	gyroA.open("/dev/ttyUSB0", 115200);
	gyroA.start();
	gyroState gy_state;

	gps gpsA;
	gpsA.open("/dev/ttyUSB0", 38400);
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
	
	{
	bool stateValid = gyroA.get_last_state(gy_state);
	while( (!stateValid) )
	{
		std::cout << "Waiting For IMU" << std::endl;
		usleep(1e5);
		stateValid = gyroA.get_last_state(gy_state);
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
			printf("sat: %i\tcog: %0.2f\t%0.8f, %0.8f\t//\t%0.8f, %0.8f\n", state.num_sat, state.courseoverground, state.lat, state.lon, waypointLat[i], waypointLon[i]);
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
			double angle_to_target = targetvector - (M_PI_2 - state.courseoverground * M_PI / 180.0);

			std::cout << "Angle to go (pre lidar): " << angle_to_target << " rad" << std::endl;
			
			if(gyroA.get_last_state(gy_state)&& fabsf(angle_to_target)<M_PI/4.0)
			{
				float start_yaw =convertyaw(gy_state.rpy[2]);
				float end_yaw = start_yaw+angle_to_target;
				float current_yaw = start_yaw;		
				float the_diff = fmodf((end_yaw - current_yaw)+M_PI,2*M_PI)-M_PI;		
				while(fabsf(the_diff)>M_PI/32.0)
				{
					gyroA.get_last_state(gy_state);					
					the_diff = fmodf((end_yaw - convertyaw(gy_state.rpy[2]))+M_PI,2*M_PI)-M_PI;					
					if( the_diff > 0)
					{
						motors.setMotorPWM(40,-40,40,-40);	
					}
					else
					{
						motors.setMotorPWM(-40,40,-40,40);
					}
				}  
			}
			else	
			{
				if(angle_to_target > 0)
				{
					//r 140, l60
					motors.setMotorPWM(140, 60, 140, 60);
					// Turn left
				}
				else
				{
					//r 20, l 140
					motors.setMotorPWM(20, 140, 20, 140);
					// Turn right
				}
			}
				while(true)
				{
					gyroState state;
					if(gyroA.get_last_state(state))
					{
						printf("<r,p,y>: <%0.2f,%0.2f,%0.2f>\tball: %0.2f\tyawrate: %0.2f\n", state.rpy[0], state.rpy[1], state.rpy[2], state.balloffset, state.yawrate);
					}
					else
					{
						std::cout << "Error getting gyro state" << std::endl;
					}
					usleep(1e5);
				}
/*
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
*/

			//got the angle we decided
			//motors.set_vel_vec(sin(angle_to_target),cos(angle_to_target));
			usleep(1e3);
		}
		distance = lambert_distance(state, target);
		std::cout << "Waypoint " << i << " hit at final distance " << distance << " m" << std::endl;
	}

	motors.setLight(MC_LIGHT_STEADY);
}
