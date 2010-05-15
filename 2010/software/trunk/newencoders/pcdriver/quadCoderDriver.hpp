#ifndef QUADCODERDRIVER
#define QUADCODERDRIVER

#include "ArduinoInterface.h"
#include "ArduinoCmds.hpp"

class quadCoderDriver
{
	public:
	quadCoderDriver();
	quadCoderDriver(byte coder_name);

	bool getEncoderState(new_encoder_pk_t& out);
	bool getEncoderVel(double& rvel, double& lvel);
	bool resetCount();

	private:
	ArduinoInterface ai;

	int ifname;
	bool m_connected;
};

#endif
