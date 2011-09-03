#include "IMU_Control.hpp"
#include <iostream>
const double IMU_Control::min_update=1;//shortest IMU correct interval [s]
const float IMU_Control::gps_drift=.1;//estimated drift threshold [m/s]
const float IMU_Control::imu_drift=.1;//estimated drift threshold (set to a number higher than the drift rate of imu) [deg/s]


void IMU_Control::start(void)
{
	init();
	while(true)
	{
		update();
		usleep(update_interval*1e6);
	}
}

float IMU_Control::sub_deg(float a,float b)
{
	return fmodf(a-b+180.0,360.0)-180.0;	
}

bool IMU_Control::heading(double& correctedHeading)
{
	double gyro_heading;
	bool retimu = gyroref.get_heading(gyro_heading);

	boost::mutex::scoped_lock lock(imu_off_mutex);
	correctedHeading = gyro_heading+imu_off;
	return retimu;
}

void IMU_Control::init(void)
{
	last_time=simpletime();

	double gps_heading;
	double imu_heading;

	bool retgps = false;
	bool retimu = false;
	
	{
		GETGPSIMUDATA:
		retgps = gpsref.get_heading(gps_heading);
		retimu = gyroref.get_heading(imu_heading);
		if(!(retgps && retimu))
		{
			usleep(1e5);
			goto GETGPSIMUDATA;
		}
	}

	imu_off = sub_deg(gps_heading,imu_heading);
	if (update_interval<min_update){
		std::cout<<"Heading correction is set to be attempted faster than the maximum speed setting. This is stupid."<<std::endl;
	}
}

void IMU_Control::update(void)
{	
	double gps_speed;
	bool retgpsspeed = gpsref.get_speed(gps_speed);

	double gps_heading;
	bool retgpsheading = gpsref.get_heading(gps_heading);
	
	double internalHeading;
	bool retinternal = heading(internalHeading);
	
	if(gps_speed>gps_drift){
		double elapsed=simpletime()-last_time;
		float error=sub_deg(gps_heading,internalHeading);
		if (elapsed<min_update) return;
		if (fabsf(error)<imu_drift*elapsed)
		{
			boost::mutex::scoped_lock lock(imu_off_mutex);
			imu_off=imu_off+std::signbit(error)*imu_drift*elapsed;
		}
		else
		{
			boost::mutex::scoped_lock lock(imu_off_mutex);
			imu_off=imu_off+error;
		}
	}
	return;
}
