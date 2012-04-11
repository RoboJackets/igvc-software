#include <math.h>
#include <sys/time.h>
#include "OSMC_4wd_driver.hpp"
#include "IMU_Control.hpp"
#include "gps.hpp"
#include "gps_common.hpp"
#include "MagnetometerTracking.hpp"
#include "EncoderTracking.hpp"
#define DIFFERENCE_THRESHOLD_POS 5
#define DIFFERENCE_THRESHOLD_ORIENTATION 1.5

#define ENCODER_COE 0.5
#define MAG_COE 0.5
#define GPS_COE 0.5

class RobotPosition
{
double x, y, z;
double angle, bearing, initBearing;
double coeMag, coeEncoder, coeIMU, coeGPS;
bool enabledMag, enabledEncoder, enabledIMU, enabledGPS;
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
RobotPosition(OSMC_4wd_driver * driver,gps& gpsObject);
static RobotPosition getInstance();
void init(OSMC_4wd_driver * driver,gps& gpsObject,IMU_Control& imuObject);
void init(OSMC_4wd_driver * driver,gps& gpsObject);
void update();
double getX();
double getY();
double getAngle();
double getBearing();
int getTimeElapsed();
void disableGPS();
void disableIMU();
void disableMag();
void disableEncoders();
void enableGPS(double coe);
void enableMag(double coe);
void enableEncoders(double coe);
void enableIMU(double coe);
private:
void updatePosition();
void updateOrientation();
};
