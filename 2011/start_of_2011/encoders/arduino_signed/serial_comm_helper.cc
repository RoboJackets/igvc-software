#include "WProgram.h"
#include "serial_comm_helper.hpp"

bool serialReadBytesTimeout(byte len, byte* msg)
{
	unsigned long t1 = millis();

	do
	{
		if(Serial.available() >= len)
		{
			for(int i = 0; i < len; ++i)
			{
				msg[i] = Serial.read();
			}
			return true;//we got the message
		}

	} while( (millis() - t1) < TIMEOUT_LENGTH_MILLIS);

	//otherwise timeout - flush link
	if(Serial.available() > 0)
	{
		delay(TIMEOUT_LENGTH_MILLIS);//let laptop know to reset link
		Serial.flush();//reset link
	}
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
