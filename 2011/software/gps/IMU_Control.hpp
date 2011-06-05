#ifndef IMU_CONTROL_HPP
#define IMU_CONTROL_HPP

#include <cmath>

#include "gps.hpp"
#include "gyro.hpp"

/*
need:
float imu_heading()//[deg,bearing]
float gps_heading()//[deg,bearing]
float gps_speed()//[m/s]
double time()//current time [s]

Call start() in a new thread
Call heading() to get the heading
*/
class IMU_Control
{

	IMU_Control(gps& g, gyro& i) : gpsref(g), gyroref(i), imu_off(0), update_interval(0)
	{

	}
	
	void start();
	float heading();
	void init();
	void update();

	private:

	inline double simpletime()
	{
		timeval now;
		gettimeofday(&now, NULL);
		return double(now.tv_sec) + double(now.tv_usec) / 1e6;
	}

	gps& gpsref;
	gyro& gyroref;

	double last_time;// last time we updated [s]
	static const double min_update_wait;//shortest IMU correct interval [s]
	static const float gps_drift;//estimated drift threshold [m/s]
	static const float imu_drift;//estimated drift threshold (set to a number higher than the drift rate of imu) [deg/s]
	float imu_off;// [deg]
	double update_interval;//update interval [s]

	float sub_deg(float a,float b);
};

#endif
