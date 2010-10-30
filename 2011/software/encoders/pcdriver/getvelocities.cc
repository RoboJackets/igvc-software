#include "quadCoderDriver.hpp"
#include <sys/time.h>
#include <stdio.h>

int main()
{
double right_vel;
double left_vel;
double t1;
FILE * ofile;
bool iserror;
timeval time;

quadCoderDriver qD(ENCODER_IF_FOR_RIGHT_BOARD);

ofile = fopen("outputfile.txt", "w");

for (int datapoint=1; datapoint <= 500; datapoint++) 
{
	iserror = qD.getEncoderVel(right_vel,left_vel);
	// Finds time		
	gettimeofday(&time, NULL);
	t1=time.tv_sec+(time.tv_usec/1000000.0);
	
	if (~iserror)
	{		
		// Writes time, right_vel and left_vel to a file
		fprintf(ofile, "time:%f\nr_vel:%f\nl_vel%f\n", t1, right_vel, left_vel);
	}
}
fclose(ofile);

return 0;

}
