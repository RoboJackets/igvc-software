
#ifndef OSMC_DRIVER
#define OSMC_DRIVER

class OSMC_driver
{

public:
	bool arduinoCheck();
	void setPwm(int pwm);
	void setMotorsPwm(int pwmLeft, int pwmRight);
	void stopMotors();
	void checkPwm(int pwm);
	void checkPwm(int pwmLeft, int pwmRight);

private:
	const static int maxPwm;
	const static int minPwm;

};

#endif
