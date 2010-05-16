#include "ArduinoInterface.h"
#include "ArduinoCmds.hpp"

#include "OSMC_driver.hpp"

class OSMC_4wd_driver
{
	public:

	OSMC_4wd_driver(const byte FORosmc, const byte FORcoder, const byte AFTosmc, const byte AFTcoder);
	//~OSMC_4wd_driver();

	//set motor
	bool setMotorPWM(const int FR, const int FL, const int BR, const int BL);//must be 0 - 255, sign is pos / rev
	bool setMotorVel_pd(const double FR, const double FL, const double BR, const double BL);

	//update pd
	bool update_pd();

	//interog
	bool getMotorVel();
	bool getLastPWMSent();

	private:

	volatile bool m_connected;

	OSMC_driver FOR;
	OSMC_driver AFT;
};
