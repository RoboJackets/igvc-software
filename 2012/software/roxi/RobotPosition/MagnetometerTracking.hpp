#include <math.h>
#include "OSMC_4wd_driver.hpp"

class MagnetometerTracking
{
private:
double bearing;
double initialBearing;
OSMC_4wd_driver * osmc;
double angle;
double XV;
double YV;

public:
MagnetometerTracking(OSMC_4wd_driver * driver);

void update();


void reset();

double getBearing();
};
