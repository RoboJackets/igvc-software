#ifndef QUADCODERDRIVER_SIGNED
#define QUADCODERDRIVER_SIGNED

#include "ArduinoInterface.h"
#include "ArduinoCmds.hpp"

class quadCoderDriver_signed
{
	public:
	quadCoderDriver_signed();
	quadCoderDriver_signed(byte coder_name);

	bool getEncoderState(new_encoder_pk_t& out);
	bool getEncoderVel(double& vel_l, double& vel_r);
	bool getEncoderDist(double& dist_l, double& dist_r);
	bool resetCount();

	private:
	ArduinoInterface ai;

	int ifname;
	bool m_connected;
};

#endif
