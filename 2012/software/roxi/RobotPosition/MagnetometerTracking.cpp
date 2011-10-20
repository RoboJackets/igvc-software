#include <math.h>
#include "OSMC_4wd_driver.hpp"

class MagnetometerTracking
{
private:
double bearing;
double initialBearing;
OSMC_4wd_driver osmc;
int angle;
int XV;
int YV;

public:
IMUTracking(OSMC_4wd_driver driver)
{
	osmc=driver;
	bearing=osmc.GetMagnetometerHeading(angle, XV, YV);
	initialBearing=bearing;
}

void update()
{
	bearing=osmc.GetMagnetometerHeading(angle, XV, YV);
}


void reset()
{
	bearing=initialBearing;
}

int getBearing()
{
	return bearing;
}
};
