#include "WProgram.h"
#include "serial_comm_helper.hpp"

bool serialReadBytesTimeout(int len, byte* msg)
{
	//Maybe return imediatly if there is no data availible
	//Only wait if there is a partial connection
	if(Serial.available() == 0)
	{
		return false;
	}

	unsigned long t1 = millis();

	while( (millis() - t1) < TIMEOUT_LENGTH_MILLIS)
	{
		if(Serial.available() >= len)
		{
			/*
			if(msg == NULL)
			{
				msg = (byte*)malloc(len);
			}
			*/
			for(int i = 0; i < len; i++)
			{
				msg[i] = Serial.read();
			}
			return true;//we got the message
		}
	}

	//otherwise timeout - flush link
	Serial.flush();
	return false;
}

void serialPrintBytes(void *data, int numBytes)
{
	for (int i = 0; i < numBytes; i++)
	{
		Serial.print(((unsigned char *)data)[i], BYTE);
	}
}
