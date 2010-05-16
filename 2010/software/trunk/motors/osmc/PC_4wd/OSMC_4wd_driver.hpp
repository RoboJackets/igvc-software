#include "ArduinoInterface.h"
#include "ArduinoCmds.hpp"

#include "OSMC_driver.hpp"

class OSMC_4wd_driver
{
	public:

	OSMC_4wd_driver(byte FORosmc, byte FORcoder, byte AFTosmc, byte AFTcoder);
	//~OSMC_4wd_driver();

	

	private:

	volatile bool m_connected;

	OSMC_driver FOR;
	OSMC_driver AFT;
};
