#include <unistd.h>
#include <string.h>
//#include <sys/socket.h>
//#include <sys/types.h>
//#include <sys/select.h>
#include <sys/time.h>
//#include <netinet/in.h>
//#include <arpa/inet.h>
#include <stdlib.h>
#include <math.h>
//#include <termios.h>

#include <vector>
using namespace std;

#include "urg_laser.h"



class URGLaserDriver {
public:

	// Constructor;
	URGLaserDriver();
	// Destructor
	~URGLaserDriver();

	// Implementations of virtual functions
	int Setup();
	int Shutdown();


private:
	// Main function for device thread.
	virtual void Main();

	urg_laser_readings_t * Readings;
	urg_laser Laser;


	bool UseSerial ;
	int BaudRate ;
	char * Port ;
};


////////////////////////////////////////////////////////////////////////////////
// Constructor.  Retrieve options from the configuration file and do any
// pre-Setup() setup.
URGLaserDriver::URGLaserDriver()

{
	Readings = new urg_laser_readings_t;

	UseSerial=false;
	BaudRate = 115200; // 011100001000000000
	Port = "/dev/ttyACM0";
	
    return;
}

URGLaserDriver::~URGLaserDriver()
{
	delete Readings;
}

////////////////////////////////////////////////////////////////////////////////
// Set up the device.  Return 0 if things go well, and -1 otherwise.
int URGLaserDriver::Setup() {
	//config data
	if(Laser.Open(Port,UseSerial,BaudRate) < 0)
	{
		printf("ERROR \n");
		return -1;
	}

 	Main();


    return(0);
}


////////////////////////////////////////////////////////////////////////////////
// Shutdown the device
int URGLaserDriver::Shutdown() {

 

  Laser.Close();

  return(0);
}




////////////////////////////////////////////////////////////////////////////////
// Main function for device thread
void URGLaserDriver::Main()
{
	int min_i =0;
	int max_i =769;
	int ranges_count = max_i - min_i;
	float ranges[MAX_READINGS];
	
	// The main loop; interact with the device here
	for(;;)	{

		// update device data
		int r = Laser.GetReadings(Readings);
		
		// check for data
		if( r )	printf("readings error: %d \n",r);
		else
		{

			for (int i = 0; i < ranges_count; ++i)
			{
				//ranges[i] = Readings->Readings[i+min_i] < 20 ? (4095) : (Readings->Readings[i+min_i]);
				//ranges[i]/=1000;
				ranges[i] = Readings->Readings[i+min_i];
			}
		
			for (int i = 0; i < ranges_count; ++i)
			{
				printf("Data %f\n",ranges[i]);
			}
		
		}
		
    	// Sleep )
    	usleep(100000);
    	printf("end line \n");

	}
}


int main(){
	URGLaserDriver L;
	L.Setup();
}

