
#ifndef OSMC_DRIVER
#define OSMC_DRIVER

class OSMC_driver
{

public:
	~OSMC_driver();
	bool arduinoCheck();
	void setPwm(byte pwm, byte dir);
	void setMotorsPwm(byte pwmLeft, byte dirLeft, byte pwmRight, byte dirRight);
	void goTurn(int degree, byte dir);
	void goForward(double dist, byte pwm, byte dir);
	void stopMotors();
	void checkPwm(byte pwm, byte dir);
	void checkPwm(byte pwmLeft, byte pwmRight);
	float readEncoder();
	void encoderLoop(float totalDist);
	void goForwardOld(float totalDist, byte pwm, byte dir);

private:
	const static int maxPwm;
	const static int minPwm;

};

#endif
