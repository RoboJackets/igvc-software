#include "ArduinoInterface.h"
#include "ArduinoCmds.hpp"

#include "OSMC_driver.hpp"

class OSMC_4wd_driver
{
	public:

	OSMC_4wd_driver(const byte FORosmc, const byte FORcoder, const byte AFTosmc, const byte AFTcoder);
	//~OSMC_4wd_driver();

	//set motor
	bool setMotorPWM(const int FR, const int FL, const int BR, const int BL);//must be -255 <-> 255, larger input trucated to 255, sign is pos / rev, true on failure
	bool setMotorPWM(const byte FRdir, const byte FRmag, const byte FLdir, const byte FLmag, const byte BRdir, const byte BRmag, const byte BLdir, const byte BLmag);//must be 0 - 255, dir is pos / rev, true on failure
	void setMotorVel_pd(const double FR, const double FL, const double BR, const double BL);

	//update pd
	bool update_pd();

	//interog
	bool getEncoderVel(double& FL, double& FR, double& BL, double& BR);
	bool getLastPWMSent();

	private:

	volatile bool m_connected;

	OSMC_driver FOR;
	OSMC_driver AFT;
};
