#ifndef QUADCODERDRIVER
#define QUADCODERDRIVER

#include "ArduinoInterface.h"
#include "ArduinoCmds.hpp"

class quadCoderDriver
{
	public:
	quadCoderDriver();

	new_encoder_pk_t getEncoderState();

	private:
	ArduinoInterface ai;

	bool m_connected;
};

#endif
