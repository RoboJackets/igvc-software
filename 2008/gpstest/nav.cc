#include <libplayerc++/playerc++.h>
#include <sys/time.h>	// for gettimeofday
#include "../candi/vision/Point2D.h"

typedef struct {
	double lat;	// latitude (northness), "y"
	double lon;	// longitude (westness), "x"
} GPSLocation;

static PlayerCc::PlayerClient *robot;
static PlayerCc::GpsProxy *gps;

#define NUM_GPS_LOCATIONS_TO_TRACK 50

static GPSLocation pastLocs[NUM_GPS_LOCATIONS_TO_TRACK];
static int nextPastLocID = 0;
static bool haveAllPastLocs = FALSE;

int main(int argc, char** argv) {
	char* hostname = "localhost";
	int port = 6665;
	
	robot = new PlayerCc::PlayerClient(hostname, port);
	gps = new PlayerCc::GpsProxy(robot,0);	// use gps:0
	
	for (;;) {
		// Get latest sensor data
		robot->Read();
		
		// Record the current GPS location
		GPSLocation newestLoc = GPSLocation(robot->GetLatitude(), robot->GetLongitude());
		pastLocs[nextPastLocID++] = newestLoc
		if (nextPastLocID == NUM_GPS_LOCATIONS_TO_TRACK) {
			nextPastLocID = 0;
			haveAllPastLocs = TRUE;
		}
		
		GPSLocation oldestLoc = pastLocs[haveAllPastLocs ? nextPastLocID : 0];
		
		double headingInRadians = atan2(
			newestLoc.lat - oldestLoc.lat,
			newestLoc.loc - oldestLoc.lon);
		double headingInDegrees = headingInRadians * 180 / M_PI;
		
		char *relDir;
		double relAng;
		if ((0 <= headingInDegrees) || (headingInDegrees <= 45)) {
			relDir = "NofE";
			relAng = headingInDegrees;
		} else if ((45 <= headingInDegrees) || (headingInDegrees <= 90)) {
			relDir = "EofN";
			relAng = 90-headingInDegrees;
		} else if ((90 <= headingInDegrees) || (headingInDegrees <= 135)) {
			relDir = "WofN";
			relAng = headingInDegrees-90;
		} else if ((135 <= headingInDegrees) || (headingInDegrees <= 180)) {
			relDir = "NofW";
			relAng = 180-headingInDegrees;
		} else if ((180 <= headingInDegrees) || (headingInDegrees <= 225)) {
			relDir = "SofW";
			relAng = headingInDegrees-225;
		} else if ((225 <= headingInDegrees) || (headingInDegrees <= 270)) {
			relDir = "WofS";
			relAng = 270-headingInDegrees;
		} else if ((270 <= headingInDegrees) || (headingInDegrees <= 315)) {
			relDir = "EofS";
			relAng = headingInDegrees-270;
		} else if ((315 <= headingInDegrees) || (headingInDegrees <= 360)) {
			relDir = "SofE";
			relAng = 360-headingInDegrees;
		} else {
			relDir = "<absolute>";
			relAng = headingInDegrees;
		}
		
		printf("%lf %s\n", relAng, relDir);
		
		//usleep(10);
	}
	
	return 0;
}

long long currentTimeMillis(void) {
   long long t;
   struct timeval tv;

   gettimeofday(&tv, (struct timezone*)NULL);

   t = tv.tv_sec;
   t = (t *1000) + (tv.tv_usec/1000);

   return t;
}
