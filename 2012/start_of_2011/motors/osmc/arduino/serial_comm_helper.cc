#include "WProgram.h"
#include "serial_comm_helper.hpp"

bool serialReadBytesTimeout(byte len, byte* msg)
{
	//Maybe return imediatly if there is no data availible
	//Only wait if there is a partial connection
	//if(Serial.available() == 0)
	//{
	//	return false;
	//}

	unsigned long t1 = millis();

	do
	{
		if(Serial.available() >= len)
		{
			/*
			if(msg == NULL)
			{
				msg = (byte*)malloc(len);
			}
			*/
			for(int i = 0; i < len; ++i)
			{
				msg[i] = Serial.read();
			}
			return true;//we got the message
		}

	} while( (millis() - t1) < TIMEOUT_LENGTH_MILLIS);

	//otherwise timeout - flush link
	Serial.flush();
	return false;
}

void serialPrintBytes(void *data, size_t numBytes)
{
//	for (int i = 0; i < numBytes; i++)
//	{
//		Serial.print(((unsigned char *)data)[i], BYTE);
//	}
	Serial.write((byte*)data, numBytes);
}
