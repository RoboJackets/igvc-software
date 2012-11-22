
#ifndef OSMC_DRIVER
#define OSMC_DRIVER

class OSMC_driver
{

public:
	~OSMC_driver();
	bool arduinoCheck();
	void setPwm(byte pwm, byte dir);
	void setMotorsPwm(byte pwmLeft, byte dirLeft, byte pwmRight, byte dirRight);
	void stopMotors();
	void checkPwm(byte pwm, byte dir);
	void checkPwm(byte pwmLeft, byte pwmRight);

private:
	const static int maxPwm;
	const static int minPwm;

};

#endif
