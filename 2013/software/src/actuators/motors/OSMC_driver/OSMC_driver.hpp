
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
	void checkPwm(char pwm, char dir);
	void checkPwm2(char pwmLeft, char pwmRight);
	float readEncoder();
	void encoderLoop(float totalDist);
	void goForwardOld(float totalDist, char pwm, char dir);

private:
	const static char maxPwm;
	const static char minPwm;

};

#endif
