#ifndef QUADCODERDRIVER
#define QUADCODERDRIVER

#include "ArduinoInterface.h"
#include "ArduinoCmds.hpp"

class quadCoderDriver
{
	public:
	quadCoderDriver();
	bool getEncoderVel(double& rvelFWD, double& lvelFWD,double& rvelAFT, double& lvelAFT);
	bool resetCount(byte iface);


	private:
	bool getEncoderVel(ArduinoInterface& ai, double& rvel, double& lvel);
	bool getEncoderState(ArduinoInterface& ai, new_encoder_pk_t& out);
	bool resetCount(ArduinoInterface& ai);

	ArduinoInterface fwd;
	ArduinoInterface aft;

	bool m_connected;
};

#endif
