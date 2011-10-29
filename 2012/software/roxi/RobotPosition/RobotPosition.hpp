#include <math.h>
#include "OSMC_4wd_driver.hpp"
#include "IMU_Control.hpp"
#include "gps.hpp"
#include "MagnetometerTracking.hpp"
#include "EncoderTracking.hpp"
class RobotPosition
{
double x;
double y;
double z;
double angle;
EncoderTracking encoder(OSMC_4wd_driver *);
MagnetometerTracking magnetometer(OSMC_4wd_driver *);
gps * gpsA;
IMU_Control * IMU;
public :
RobotPosition(OSMC_4wd_driver * driver,gps& gpsObject,IMU_Control& imuObject);
void update();

};
