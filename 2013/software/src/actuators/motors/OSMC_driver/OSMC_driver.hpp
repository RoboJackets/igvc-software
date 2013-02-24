
#ifndef OSMC_DRIVER
#define OSMC_DRIVER

#include "serial/ASIOSerialPort.h"

using namespace std;

class OSMC_driver
{

public:

	enum Direction {
	    LEFT,
	    RIGHT,
	    FORWARD,
	    BACKWARD
	};

    OSMC_driver();
	~OSMC_driver();
	bool arduinoCheck();
	void setPwm(char pwm, char dir);
	void setMotorsPwm(char pwmLeft, char dirLeft, char pwmRight, char dirRight);
	void goTurn(int degree, char dir);
	void goForward(double dist, char pwm, char dir);
	void stopMotors();
	double readEncoder();
	void encoderLoop(double totalDist);
	void goForwardOld(double totalDist, char pwm, char dir);
	void setRightLeftPwm(char pwmRight, char dirRight, char pwmLeft, char dirLeft);
	char adjustSpeedRight(char pwm, char dir);
	char adjustSpeedLeft(char pwm, char dir);
	char adjustDirLeft(char dirLeft);
	void turn(double radius, int pwm, Direction dir);
	string intToString(int input);
	string doubleToString(double input);
	void forward(char pwm, char dir, double dist);

private:
	const static double WHEELBASE = 0.762; //meters
	ASIOSerialPort arduinoLeft;
    ASIOSerialPort arduinoRight;
    ASIOSerialPort arduinoEncoder;

};

#endif
