#include <math.h>
#include "OSMC_4wd_driver.hpp"

class EncoderTracking
{
double x;
double y;
double angle;
const static double WHEEL_RADIUS=1;
const static double WHEEL_BASE=1;//distance between wheels 
const static int CYCLES_PER_ROTATION=1;
OSMC_4wd_driver * osmc;
public :
EncoderTracking(OSMC_4wd_driver * driver);

EncoderTracking();

void update();

void reset();

void setTo(double X, double Y, double Angle);

double getX();

double getY();

double getAngle();
};
