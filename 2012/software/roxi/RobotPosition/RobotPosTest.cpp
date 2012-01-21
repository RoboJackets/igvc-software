#include <math.h>
#include <sys/time.h>
#include <stdio.h>
#define MILLISECONDS_PER_CYCLE 3000
#define TIME_CONSTANT 1
#define SAMPLE_PERIOD 50
int main()
{
	double x=0;
double y=0;
double z=0;
double angle=M_PI/2;
int updatesSinceReset=0;
double filterCoefficient=TIME_CONSTANT/(TIME_CONSTANT+SAMPLE_RATE);
struct timeval initialTime, currentTime;
	updatesSinceReset=0;
	x=0;	
	y=0;
	z=0;
	angle=M_PI/2;
	gettimeofday(&initialTime,NULL);
for(int i =0;i<1000000;i++)
{
	gettimeofday(&currentTime,NULL);
	double xEncoder=
	double timeDifference=(currentTime.tv_sec-initialTime.tv_sec)*1000.0+(currentTime.tv_usec-initialTime.tv_usec)/1000.0;
	x+=xEncoder*(1-filterCoefficient)+xGPS*(filterCoefficient);
	y+=yEncoder*(1-filterCoefficient)+yGPS*(filterCoefficient);
	printf("X:%f   ;   Y:%f    time:%f\n",x,y,timeDifference);
	}
	return 0;
}
