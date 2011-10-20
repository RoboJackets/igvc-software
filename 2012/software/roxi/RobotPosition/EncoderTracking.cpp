#include <math.h>
#include "OSMC_4wd_driver.hpp"

class encoderTracking
{
private:
double x;
double y;
double angle;
const double WHEEL_RADIUS=1;
const double WHEEL_BASE=1;//distance between wheels 
const int CYCLES_PER_ROTATION=1;
OSMC_4wd_driver osmc;



public :
encoderTracking(OSMC_4wd_driver driver):x(0),y(0),angle(M_PI/2)
{
osmc=driver;
}

void update()
{
	int lastRightCycles;//get data from OSMC
	int lastLeftCycles;//get data from OSMC
	double distanceRight=wheelRadius/cyclesPerRotation*lastRightCycles;
	double distanceLeft=wheelRadius/cyclesPerRotation*lastLeftCycles;
	
	double distanceCenter=(distanceRight+distanceLeft)/2;
	//Counterclockwise currently set as positive - should possibly change depending on gyro/other code
	angle+=(distanceRight-distanceLeft)/(2*wheelBase);
	x+=distanceCenter*sin(angle);
	y+=distanceCenter*cos(angle);
	 
}

void reset()
{
	x=0;
	y=0;
	angle=M_PI/2;
}

int getX()
{
	return x;
}

int getY()
{
	return y;
}

int getAngle()
{
	return angle;
}
};
