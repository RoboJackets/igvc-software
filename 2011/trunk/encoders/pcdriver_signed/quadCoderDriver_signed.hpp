#ifndef QUADCODERDRIVER_SIGNED
#define QUADCODERDRIVER_SIGNED

#include "ArduinoInterface.h"
#include "ArduinoCmds.hpp"

class quadCoderDriver_signed
{
	public:
	quadCoderDriver_signed();
	quadCoderDriver_signed(byte coder_name);

	bool getEncoderState(new_encoder_single_pk_t& out);
	bool getEncoderVel(double& vel);
	bool resetCount();

	private:
	ArduinoInterface ai;

	int ifname;
	bool m_connected;
};

#endif
