#include "RobotPosition.hpp"

RobotPosition::RobotPosition(OSMC_4wd_driver* driver, gps& gpsObject, IMU_Control& imuObject)
{
	magnetometer=new MagnetometerTracking(driver);
	updatesSinceReset=0;
	x=0;	
	y=0;
	z=0;
        coeMag=0; coeEncoder=0; coeIMU=0; coeGPS=1;
	//State for all sensors but GPS initially off(GPS is required)
	enabledMag=false; enabledEncoder=false; enabledIMU=false; enabledGPS=true;
	(*magnetometer).update();
	initBearing=(*magnetometer).getBearing();
	encoder=new EncoderTracking(driver);
	(*encoder).setTo(0,0,450-initBearing);
	IMU=&imuObject;	
	gpsA=&gpsObject;
	(*gpsA).get_last_state(gpsFirstState);
	timeElapsed=0;
	gettimeofday(&initialTime,NULL);
}

RobotPosition::RobotPosition(double X, double Y, double Angle){
	x=X;
	y=Y;
	angle=Angle;
}

RobotPosition RobotPosition::getInstance(){
	return instance;
}

void RobotPosition::update()
{
	gettimeofday(&currentTime,NULL);
	timeElapsed=(currentTime.tv_sec-initialTime.tv_sec)*1000+(currentTime.tv_usec-initialTime.tv_usec)/1000;
	updatePosition();
	updateOrientation();
}

void RobotPosition::updatePosition(){
	GPSState lastState;
	(*gpsA).get_last_state(lastState);
	double xGPS;
	double yGPS;
	lambert_distance_xy(gpsFirstState,lastState,&xGPS,&yGPS);
	if(abs(xGPS-x)>=DIFFERENCE_THRESHOLD_POS || abs(yGPS-y)>=DIFFERENCE_THRESHOLD_POS)
		disableGPS();
	else if(!enabledGPS)
		enableGPS(GPS_COE);

	double xEncoder=(*encoder).getX();
	double yEncoder=(*encoder).getY();
	if(abs(xEncoder-x)>=DIFFERENCE_THRESHOLD_POS || abs(yEncoder-y)>=DIFFERENCE_THRESHOLD_POS)
		disableEncoders();
	else if(!enabledEncoder)
		enableEncoders(ENCODER_COE);

	x=xGPS*coeGPS+xEncoder*coeEncoder;
	y=yGPS*coeGPS+yEncoder*coeEncoder;

}

void RobotPosition::updateOrientation(){
	double magBearing=(*magnetometer).getBearing();
	if(abs(magBearing-bearing)>=DIFFERENCE_THRESHOLD_ORIENTATION)
		disableMag();
	else if(!enabledMag)
		enableMag(MAG_COE);
	double encoderBearing=(*encoder).getBearing();
	if(abs(encoderBearing-bearing)>=DIFFERENCE_THRESHOLD_ORIENTATION)
		disableMag();
	else if(!enabledMag)
		enableMag(ENCODER_COE);

	bearing=coeMag*magBearing+coeEncoder*encoderBearing;
	
}

void RobotPosition::disableGPS(){
enabledGPS=false;
coeGPS=0;
coeEncoder=1;
}

void RobotPosition::disableIMU(){
enabledIMU=false;
coeIMU=0;
coeMag=1;
}

void RobotPosition::disableMag(){
enabledMag=false;
coeMag=0;
coeIMU=1;
}

void RobotPosition::disableEncoders(){
enabledEncoder=false;
coeEncoder=0;
coeGPS=1;
}

void RobotPosition::enableGPS(double coe){
enabledGPS=true;
coeGPS=coe;
coeEncoder=1-coe;
}

void RobotPosition::enableIMU(double coe){
enabledIMU=true;
coeIMU=coe;
coeMag=1-coe;
}

void RobotPosition::enableMag(double coe){
enabledMag=true;
coeMag=coe;
coeIMU=1-coe;
}

void RobotPosition::enableEncoders(double coe){
enabledEncoder=true;
coeEncoder=coe;
coeGPS=1-coe;
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
