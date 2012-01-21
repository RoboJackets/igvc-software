#include "RobotPosition.hpp"
RobotPosition::RobotPosition(OSMC_4wd_driver* driver, gps& gpsObject, IMU_Control& imuObject)
{
	updatesSinceReset=0;
	x=0;	
	y=0;
	z=0;
	(*magnetometer).update();
	initBearing=(*magnetometer).getBearing();
	posFilterCoefficient=POS_TIME_CONSTANT/(POS_TIME_CONSTANT+SAMPLE_PERIOD);
	orientationFilterCoefficient=ORIENTATION_TIME_CONSTANT/(ORIENTATION_TIME_CONSTANT+SAMPLE_PERIOD);
	encoder=new EncoderTracking(driver);
	IMU=&imuObject;
	magnetometer=new MagnetometerTracking(driver);
	gpsA=&gpsObject;
	(*gpsA).get_last_state(gpsFirstState);
	timeElapsed=0;
	gettimeofday(&initialTime,NULL);
}

void RobotPosition::update()
{
	gettimeofday(&currentTime,NULL);
	timeElapsed=(currentTime.tv_sec-initialTime.tv_sec)*1000+(currentTime.tv_usec-initialTime.tv_usec)/1000;
	GPSState lastState;
	(*gpsA).get_last_state(lastState);
	double xGPS;
	double yGPS;
	lambert_distance_xy(gpsFirstState,lastState,&xGPS,&yGPS);


	(*encoder).update();
	double xEncoder=(*encoder).getX();
	double yEncoder=(*encoder).getY();
	
	(*magnetometer).update();
	double magBearing=(*magnetometer).getBearing();
	angle=lambert_bearing(gpsFirstState,lastState)*(1-orientationFilterCoefficient)+(*encoder).getAngle()*orientationFilterCoefficient;
	x+=xEncoder*(posFilterCoefficient)+xGPS*(1-posFilterCoefficient);
	y+=yEncoder*(posFilterCoefficient)+yGPS*(1-posFilterCoefficient);
	(*encoder).setTo(x,y,angle); 
	

	
	//filtering may not be needed for bearing if magnetometer is good enough by itself - determine in testing
	bearing=magBearing*(1-orientationFilterCoefficient)+(initBearing-angle+360)*orientationFilterCoefficient;
	//bearing=magBearing;
}

/*
Gets the robot's x position in meters with respect to the starting position.
*/
double RobotPosition::getX()
{
return x;
}
/*
Gets the robot's x position in meters with respect to the starting position.
*/
double RobotPosition::getY()
{
return y;
}
/*
Gets the robot's bearing.
*/
double RobotPosition::getBearing()
{
return bearing;
}
/*
Gets the robot's angle with respect to the starting angle(starts as PI/2).
*/
double RobotPosition::getAngle()
{
return angle;
}
/*
Returns the elapsed time up to the last update in millimiters.
*/
int RobotPosition::getTimeElapsed()
{
return timeElapsed;
}
