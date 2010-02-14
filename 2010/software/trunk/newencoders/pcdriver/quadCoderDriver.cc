
#include "quadCoderDriver.hpp"

quadCoderDriver::quadCoderDriver()
{
	m_connected = false;
	ai.initLink(ENCODER_IF_BOARD);
	m_connected = true;
}

new_encoder_pk_t quadCoderDriver::getEncoderState()
{
	new_encoder_pk_t out;

	if(ai.sendCommand(ENCODER_GET_READING, NULL, 0))
	{
		return out;
	}

	byte retcmd = 0;
	byte* data = NULL;
	
	if(ai.recvCommand(retcmd, data))
	{
		return out;
	}

	memcpy(&out, data, sizeof(new_encoder_pk_t));
	delete[] data;
	return out;
}
