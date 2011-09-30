#include <math.h>

float x=0;
float y=0;
float angle=M_PI/2;
float lastRightCycles;//updated by encoders from outside
float lastLeftCycles;
const float millisecondsBetweenChecks=100;//arbitrary value currently
const float wheelRadius=1;
const float wheelBase=1;//distance between wheels 
const int cyclesPerRotation=1;

int main()
{
return 0;
}

void update()
{
	float distanceRight=wheelRadius/cyclesPerRotation*lastRightCycles;
	float distanceLeft=wheelRadius/cyclesPerRotation*lastLeftCycles;
	
	float distanceCenter=(distanceRight+distanceLeft)/2;
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
	lastRightCycles=0;
	lastLeftCycles=0;
}
