#include <math.h>
#include "OSMC_4wd_driver.hpp"

public class RobotPosition
{
private:
double x;
double y;
double z;
double angle;
EncoderTracking encoder;
MagnetometerTracking magnetometer;
gps GPS;
IMU_Control IMU;
//currently inline - will be changed later
public :
RobotPosition(OSMC_4wd_driver driver, gps gpsObject, IMU_Control imuObject)
{
IMU=imuObject;
GPS=gpsobject;
x=0;
y=0;
z=0;
angle=M_PI/2;
encoder=new encoderTracking(driver);
IMU=new IMUTracking(driver);
magnetomer=new MagnetometerTracking();
GPS=new GPSTracking();
}

update()
{
GPSTracking.update();
encoder.update();
IMU.update();
magnetometer.update();

}



}
