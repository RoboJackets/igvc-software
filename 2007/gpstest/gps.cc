#include <libplayerc++/playerc++.h>

static PlayerCc::PlayerClient *robot;
static PlayerCc::GpsProxy *gps;

int main(int argc, char** argv) {
	char* hostname = "localhost";
	int port = 6665;
	
	robot = new PlayerCc::PlayerClient(hostname, port);
	gps = new PlayerCc::GpsProxy(robot,0);	// use gps:0
	
	for (;;) {
		// Get latest sensor data
		robot->Read();
		
		// Print out current position
		printf("lat=%lf, lon=%lf, nsats=%d, qual=%d\n",
			(double) gps->GetLatitude(),
			(double) gps->GetLongitude(),
			(int) gps->GetSatellites(),
			(int) gps->GetQuality());
		
		//usleep(10);
	}
	
	return 0;
}
