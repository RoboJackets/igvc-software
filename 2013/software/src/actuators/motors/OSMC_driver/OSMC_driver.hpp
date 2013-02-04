
#ifndef OSMC_DRIVER
#define OSMC_DRIVER

class OSMC_driver
{

public:
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

private:
	const static char maxPwm;
	const static char minPwm;

};

#endif
