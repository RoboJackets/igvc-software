#include <math.h>

need:
float imu_heading()//[deg,bearing]
float gps_heading()//[deg,bearing]
float gps_speed()//[m/s]
double time()//current time [s]

Call start() in a new thread
Call heading() to get the heading



	double last_time;// last time we updated [s]
	const double min_update_wait=1;//shortest IMU correct interval [s]
	const float gps_drift=.1;//estimated drift threshold [m/s]
	const float imu_drift=.1;//estimated drift threshold (set to a number higher than the drift rate of imu) [deg/s]
	float imu_off=0;// [deg]
	double update_interval=1.0;//update interval [s]
	
	void start(void){
		init();
		while(true){
			update();
			usleep(update_interval*1e6);
		}
	}
	
	float sub_deg(float a,float b){
		return fmodf(a-b+180.0,360.0)-180.0;	
	}
	
	float heading(void){
		return imu_heading()+imu_off;
	}
	
	void init(void){
		last_time=time();
		imu_off=sub_deg(gps_heading(),imu_heading());
		
	}

	void update(void){
		
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

