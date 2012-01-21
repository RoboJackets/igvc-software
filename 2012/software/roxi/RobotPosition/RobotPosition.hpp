#include <math.h>
#include <time.h>
#include "OSMC_4wd_driver.hpp"
#include "IMU_Control.hpp"
#include "gps.hpp"
#include "gps_common.hpp"
#include "MagnetometerTracking.hpp"
#include "EncoderTracking.hpp"
#define SECONDS_PER_CYCLE 30

class RobotPosition
{
double x;
double y;
double z;
double angle;
double encoderWeight;
double gyroWeight;
double GPSWeight;
double bearing;
int updatesSinceReset;
GPSState gpsFirstState;
EncoderTracking encoder(OSMC_4wd_driver *);
MagnetometerTracking magnetometer(OSMC_4wd_driver *);
time_t initialTime;
gps * gpsA;
IMU_Control * IMU;
public :
RobotPosition(OSMC_4wd_driver * driver,gps& gpsObject,IMU_Control& imuObject);
void update();
double getX();
double getY();
double getAngle();
double getBearing();

};
