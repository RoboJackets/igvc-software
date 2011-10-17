#ifndef QUADCODERDRIVER_4WD_SIGNED
#define QUADCODERDRIVER_4WD_SIGNED

#include "ArduinoInterface.h"
#include "ArduinoCmds.hpp"

#include "quadCoderDriver_signed.hpp"

class quadCoderDriver_4wd_signed
{
	public:
	quadCoderDriver_4wd_signed();
	bool getEncoderVel(double& rvelFWD, double& lvelFWD,double& rvelAFT, double& lvelAFT);
	bool resetCount();


	private:

	quadCoderDriver_signed front;
	quadCoderDriver_signed back;

	bool m_connected;
};

#endif
