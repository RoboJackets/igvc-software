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
}

void RobotPosition::update()
{
	GPSState lastState;
	gpsA.get_last_state(&lastState);
	double xGPS;
	double yGPS;
	lambert_distance_xy(&gpsFirstState,&lastState,&xGPS,&yGPS);


	encoder.update();
	IMU.update();
	double xDeltaEncoder=encoder.getX();
	double yDeltaEncoder=encoder.getY();
	double angleDeltaEncoder=encoder.getAngle();
	magnetometer.update();
	double magBearing=magnetometer.getBearing();
	x+=(xDeltaEncoder*encoderWeight+xGPS*GPSWeights)/2;
	y+=(yDeltaEncoder*encoderWeight+yGPS*GPSWeight)/2;
	
	encoder.reset();
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
double getBearing()
{
return bearing;
}
