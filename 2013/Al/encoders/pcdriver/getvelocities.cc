//#include "quadCoderDriver.hpp"
#include "OSMC_driver.hpp"
#include <sys/time.h>
#include <fstream>
using namespace std;

// Uses OSMC driver and records velocity and time data to a text file to find taus of motors
// Kenneth Marino

int main()
{
	double right_vel;
	double left_vel;
	double t1;
	bool iserror;
	timeval time;

	//quadCoderDriver qD(ENCODER_IF_FOR_RIGHT_BOARD);
	OSMC_driver qD;

	ofstream datafile("outputfile.txt");
	qD.set_motors(80,80);

	for (int datapoint=1; datapoint <= 500; datapoint++) 
	{
		iserror = qD.getEncoderVel(right_vel,left_vel);
		// Finds time		
		gettimeofday(&time, NULL);
		t1=double(time.tv_sec)+(double(time.tv_usec)/1e6);
	
		if (~iserror)
		{		
			// Writes time, right_vel and left_vel to a file
			datafile << "time:" << t1 << endl << "r_vel:" << right_vel << endl << "l_vel:" << left_vel << endl;	
			//fprintf(ofile, "time:%f\nr_vel:%f\nl_vel:%f\n", t1, right_vel, left_vel);
		}
	}
	
	//fclose(ofile);
	datafile.close();
	return 0;

}
