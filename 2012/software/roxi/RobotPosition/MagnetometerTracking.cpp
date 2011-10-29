#include "MagnetometerTracking.hpp"

MagnetometerTracking::MagnetometerTracking(OSMC_4wd_driver * driver)
{
	osmc=driver;
	//bearing=(*osmc).GetMagnetometerHeading(angle, XV, YV);
	initialBearing=bearing;
}

void MagnetometerTracking::update()
{
	//bearing=(*osmc).GetMagnetometerHeading(angle, XV, YV);
}

void MagnetometerTracking::reset()
{
	bearing=initialBearing;
}

double MagnetometerTracking::getBearing()
{
	return bearing;
}
