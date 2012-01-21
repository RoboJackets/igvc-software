
#include "EncoderTracking.hpp"


EncoderTracking::EncoderTracking(OSMC_4wd_driver * driver)
{
x=0;
y=0;
angle=M_PI/2;
osmc=driver;
}

EncoderTracking::EncoderTracking()
{
x=0;
y=0;
angle=M_PI/2;
}

void EncoderTracking::reset()
{
	x=0;
	y=0;
	angle=M_PI/2;
}

void EncoderTracking::setTo(double X, double Y, double Angle)
{
	x=X;
	y=Y;
	angle=Angle;
}

double EncoderTracking::getX()
{
	return x;
}

double EncoderTracking::getY()
{
	return y;
}

double EncoderTracking::getAngle()
{
	return angle;
}

void EncoderTracking::update()
{
	int lastRightCycles;//get data from OSMC
	int lastLeftCycles;//get data from OSMC
	double distanceRight=WHEEL_RADIUS/CYCLES_PER_ROTATION*lastRightCycles;
	double distanceLeft=WHEEL_RADIUS/CYCLES_PER_ROTATION*lastLeftCycles;
	
	double distanceCenter=(distanceRight+distanceLeft)/2;
	//Counterclockwise currently set as positive - should possibly change depending on gyro/other code
	angle+=(distanceRight-distanceLeft)/(2*WHEEL_BASE);
	x+=distanceCenter*sin(angle);
	y+=distanceCenter*cos(angle);
}


