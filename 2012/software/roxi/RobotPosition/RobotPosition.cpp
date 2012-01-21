#include "RobotPosition.hpp"
#define updatesUntilReset 100
RobotPosition::RobotPosition(OSMC_4wd_driver* driver, gps& gpsObject, IMU_Control& imuObject)
{
	updatesSinceReset=0;
	x=0;	
	y=0;
	z=0;
	angle=M_PI/2;
	EncoderTracking encoder(driver);
	IMU=&imuObject;
	MagnetometerTracking magnetometer(driver);
	gpsA=&gpsObject;
	gpsA.get_last_state(&gpsFirstState);
	time(&initialTime);
}

void RobotPosition::update()
{
	time_t currentTime=time(NULL);
	timeDifference=difftime(initialTime,currentTime);
	GPSState lastState;
	gpsA.get_last_state(&lastState);
	double xGPS;
	double yGPS;
	lambert_distance_xy(&gpsFirstState,&lastState,&xGPS,&yGPS);


	encoder.update();
	IMU.update();
	double xEncoder=encoder.getX();
	double yEncoder=encoder.getY();
	angle=encoder.getAngle();
	magnetometer.update();
	double magBearing=magnetometer.getBearing();
	x+=xEncoder*cos(timeDifference*MATH_PI/SECONDS_PER_CYCLE)+xGPS*sin(timeDifference*MATH_PI/SECONDS_PER_CYCLE);
	y+=yEncoder*cos(timeDifference*MATH_PI/SECONDS_PER_CYCLE)+yGPS*sin(timeDifference*MATH_PI/SECONDS_PER_CYCLE);
	if(timeDifference%SECOND_PER_CYCLE<0.2)
		encoder.setTo(x,y,angle); 

	
	
	bearing=magBearing;
}

double RobotPosition::getX()
{
return x;
}
double RobotPosition::getY()
{
return y;
}
double RobotPosition::getBearing()
{
return bearing;
}
double RobotPosition::getAngle()
{
return angle;
}
