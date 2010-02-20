#ifndef QUADCODERDRIVER
#define QUADCODERDRIVER

#include "ArduinoInterface.h"
#include "ArduinoCmds.hpp"

class quadCoderDriver
{
	public:
	quadCoderDriver();

	bool getEncoderState(new_encoder_pk_t& out);

	private:
	ArduinoInterface ai;

	bool m_connected;
};

#endif
