#include "RobotPosition.hpp"

RobotPosition::RobotPosition(OSMC_4wd_driver* driver, gps& gpsObject, IMU_Control& imuObject)
{
	x=0;	
	y=0;
	z=0;
	angle=M_PI/2;
	EncoderTracking encoder(driver);
	IMU=&imuObject;
	MagnetometerTracking magnetometer(driver);
	gpsA=&gpsObject;
}

void RobotPosition::update()
{
	//GPS.update();
	//encoder.update();
	//IMU.update();
	//magnetometer.update();

}
