#include "IMU_Control.hpp"

const double IMU_Control::min_update_wait=1;//shortest IMU correct interval [s]
const float IMU_Control::gps_drift=.1;//estimated drift threshold [m/s]
const float IMU_Control::imu_drift=.1;//estimated drift threshold (set to a number higher than the drift rate of imu) [deg/s]


void IMU_Control::start(void){
	init();
	while(true){
		update();
		usleep(update_interval*1e6);
	}
}

float IMU_Control::sub_deg(float a,float b)
{
	return fmodf(a-b+180.0,360.0)-180.0;	
}

float IMU_Control::heading(void)
{
	return imu_heading()+imu_off;
}

void IMU_Control::init(void)
{
	last_time=time();

	double gps_heading;
	double imu_heading;

	bool retgps = false;
	bool retimu = false;
	
	{
		GETGPSIMUDATA:
		retgps = gps.get_heading(gps_heading);
		retimu = imu.get_heading(imu_heading);
		if(!(retgps && retimu))
		{
			usleep(1e5);
			goto GETGPSIMUDATA;
		}
	}

	imu_off = sub_deg(gps_heading,imu_heading);
	
}

void IMU_Control::update(void)
{	
	if(gps_speed()>gps_drift){
		double elapsed=time()-last_time;
		float error=sub_deg(gps_heading(),heading());
		if (elapsed<min_update) return;
		if (fabsf(error)<imu_drift*elapsed){
			imu_off=imu_off+signbit(error)*imu_drift*elapsed;
		}else{
			imu_off=imu_off+error;
		}
	}
	return;
}
