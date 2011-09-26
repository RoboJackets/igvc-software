#ifndef OSMC_4wd_driver_HPP
#define OSMC_4wd_driver_HPP

#include "ArduinoInterface.h"
#include "ArduinoCmds.hpp"

#include "OSMC_driver.hpp"

//following gnu libc tradition,
//a return value of false is success (eg, code 0)
//a return value of true is failure (eg, code nonzero)

class OSMC_4wd_driver
{
	public:
	OSMC_4wd_driver();
	OSMC_4wd_driver(const byte FORosmc, const byte FORLcoder, const byte FORRcoder, const byte AFTosmc, const byte AFTLcoder, const byte AFTRcoder);
	//~OSMC_4wd_driver();

	//set motor
	bool setMotorPWM(const int FR, const int FL, const int BR, const int BL);//must be -255 <-> 255, larger input mag truncated to 255, sign is pos / rev, true on failure
	bool setMotorPWM(const byte FRdir, const byte FRmag, const byte FLdir, const byte FLmag, const byte BRdir, const byte BRmag, const byte BLdir, const byte BLmag);//must be 0 - 255, dir is pos / rev, true on failure
	void setVel_pd(const double FR, const double FL, const double BR, const double BL);
	void setVel_pd(const double v);
	bool set_motors(const int pwm);

	//set vel from vision vector
	bool set_vel_vec(const double y, const double x);

	//update pd
	bool updateVel_pd();

	//interog
	bool getEncoderVel(double& FL, double& FR, double& BL, double& BR);
	bool getLastPWMSent(byte& FL, byte& FR, byte& BL, byte& BR);

	//smart reckoning
	//void followCirc(double radius, double vmag);

	bool setLight(const byte option);

	private:

	volatile bool m_connected;

	OSMC_driver FOR;
	OSMC_driver AFT;
	bool GetMagnetometerHeading(int& angle, int& X_Value, int& Y_Value);
};

#endif
