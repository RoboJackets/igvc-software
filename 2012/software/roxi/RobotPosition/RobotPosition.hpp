#include <math.h>
#include <sys/time.h>
#include "OSMC_4wd_driver.hpp"
#include "IMU_Control.hpp"
#include "gps.hpp"
#include "gps_common.hpp"
#include "MagnetometerTracking.hpp"
#include "EncoderTracking.hpp"
#define POS_TIME_CONSTANT 3
#define ORIENTATION_TIME_CONSTANT 3
#define SAMPLE_PERIOD 50

class RobotPosition
{
double x;
double y;
double z;
double angle;
double bearing;
double initBearing;
double posFilterCoefficient;
double orientationFilterCoefficient;
int updatesSinceReset;
int timeElapsed;
GPSState gpsFirstState;
EncoderTracking *encoder;
MagnetometerTracking *magnetometer;
timeval initialTime,currentTime;
gps * gpsA;
IMU_Control * IMU;
public :
RobotPosition(OSMC_4wd_driver * driver,gps& gpsObject,IMU_Control& imuObject);
void update();
double getX();
double getY();
double getAngle();
double getBearing();
int getTimeElapsed();
};
