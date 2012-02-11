#include <math.h>
#include <sys/time.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "OSMC_4wd_driver.hpp"
#include "IMU_Control.hpp"
#include "gps.hpp"
#include "gps_common.hpp"
#include "MagnetometerTracking.hpp"
#include "EncoderTracking.hpp"
#include "MatrixMath.hpp"

/**
Implementation of robot position with a Kalman Filter used to reduce senor noise

Estimates the process described by the following the standard equations:
	 x_k=Ax_k-1+Bu_k(+w_k-1)
x_k=robot state at time step k
-A=nxn matrix that relates the state of the previous time step to the current one 
disregarding driving input or process noise
-x_k-1=robot state at the previous time step k-1
-B=nxl matrix that relates the control input u to state x
-u_k=The control input for the robot at time step k
-(w_k-1)=Process noise(difference from motion expected from input and state due to outside disturbances) 
with a normal probability distribution p(w)~N(0,Q) ; Q=Proccess noise covariance 


	z_k= Hx_k (+ v_k)
-z_k=Measurements of state at time step k
-H=mxn matrix relating the state to the measurement of the state(constant offsets, etc.)
-x_k=robot state at time step k
-(v_k)=Sensor noise at time step k with a normal probability distribution p(w)~N(0,R) ; R=Sensor noise covariance

For equations actually used to estimate the process look in RobotPositionKalman.cpp

-For much fuller description, read:www.cs.unc.edu/~tracker/media/pdf/SIGGRAPH2001_CuorsePack_08.pdf
				   www.cs.unc.edu/~welch/kalman/media/pdf/maybeck_ch1.pdf
*/
#define CALL_PERIOD 0.05
class RobotPositionKalman
{

CvMat * x;//7x1 - posX,posY,bearing,velX,velY,accX,accY - state at step k
CvMat * z;//6x1 - state measurements at step k
CvMat * A;//3x3 - Relation of x_k-1 to x_k
CvMat * B;//3x2 - Relation of u_k(control inputs at step K) to x_k
CvMat * H;//6x3 - Relation of z_k to x_k
CvMat * K;//3x6 - gain or blending factor in x_k=x_k-1+K(z_k-x_k_apr)
CvMat * P;//3x3 - Estimate error covariance at step k
CvMat * R;//3x3 - Process noise covariance
CvMat * Q;//3x3 - Measurement noise covariance
GPSState gpsFirstState;
EncoderTracking *encoder;
MagnetometerTracking *magnetometer;
timeval initialTime,currentTime;
gps * gpsA;
IMU_Control * IMU;
public :
RobotPositionKalman(OSMC_4wd_driver * driver,gps& gpsObject,IMU_Control& imuObject);
void update(double *);
double getX();
double getY();
double getAngle();
double getBearing();
};
