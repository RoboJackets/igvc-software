#include "WProgram.h"

#include <util/delay.h>

#include "DataPacketStructs.hpp"
#include "ArduinoCmds.hpp"
#include "common_defines.hpp"

#include "serial_comm_helper.hpp"

#include "diffenc.hpp"
#include "pindefs.hpp"

extern "C" void __cxa_pure_virtual()
{
	for(;;)
	{

	}
}

void genTimestamp(long * sec, long * usec)
{
	//*sec =  global_time_sec + (millis()/1000) - arduino_time_millis/1000;
	//*usec = *sec - (global_time_usec + millis()*1000 - arduino_time_millis*1000);

	*sec = 0;
	*usec = 0;
}

int main()
{	
	init();
	Serial.begin(SERIAL_BAUD);
	Serial.flush();

	pinMode(left_encoder_pin_A, INPUT);
	pinMode(left_encoder_pin_B, INPUT);

	pinMode(right_encoder_pin_A, INPUT);
	pinMode(right_encoder_pin_B, INPUT);

	//attachInterrupt(0, encoder_logger, CHANGE);
	//attachInterrupt(1, encoder_logger, CHANGE);

	//attachInterrupt(0, leftenc_event, CHANGE);//pin 2
	//attachInterrupt(1, rightenc_event, CHANGE);//pin 3

	attachInterrupt(0, leftenc_event, RISING); // pin 2
	attachInterrupt(1, rightenc_event, RISING); // pin 3

	int32_t tx_num = 0;

	left_coder_ticks = 0;
	right_coder_ticks = 0;
	//coder_ticks = 0;
	for(;;)
	{
		//overspeed test
		//if((lastupdate - milis()) > 75)
		//
		header_t header;
		byte* indata = NULL;

		if(!serialReadBytesTimeout(PACKET_HEADER_SIZE, (byte*)&header))
		{
			continue;
		}

		if(header.size > 0)
		{
			if(header.size > 64)
			{
				continue;
			}
			indata = (byte*)malloc(header.size);
			if(!serialReadBytesTimeout(header.size, indata))
			{
				free(indata);
				indata = NULL;
				continue;
			}
		}

		switch(header.cmd)
		{
			case ARDUINO_RESET:
			{
				//Serial.println("Reset");
				break;
			}
			case ARDUINO_SET_CLOCK:
			{
				//Serial.println("Set Clock");
				break;
			}
			case ARDUINO_ID_CMD:
			{
				header_t headerOut;
				genTimestamp(&headerOut.timestamp_sec, &headerOut.timestamp_usec);

				headerOut.packetnum = tx_num;
				headerOut.cmd = ARDUINO_ID_CMD;
				headerOut.size = 1;
				//char msg = ENCODER_IF_AFT_LEFT_BOARD;
				char msg = ENCODER_IF_FOR_RIGHT_BOARD;

				Serial.write((uint8_t*)&headerOut, PACKET_HEADER_SIZE);
				Serial.print(msg, BYTE);
				tx_num++;
				break;
			}
			#if 0
			case ENCODER_GET_READING:
			{
				header_t headerOut;
				genTimestamp(&headerOut.timestamp_sec, &headerOut.timestamp_usec);

				headerOut.packetnum = tx_num;
				headerOut.cmd = ENCODER_GET_READING;
				headerOut.size = sizeof(new_encoder_pk_t);

				new_encoder_pk_t body;

				int64_t first_left = coder_ticks;
				int64_t first_right = 0;
				//delay(5);
				_delay_ms(5);
				body.pl = coder_ticks;
				body.pr = 0;

				body.dl = coder_ticks - first_left;
				body.dr = 0;
				//body.dr = right_ticks - first_right;
				//body.dr = ((PIND & 0x0C) >> 2) & (~0xFC);

				uint8_t* msg = (uint8_t*)&(body);
				Serial.write((uint8_t*)&headerOut, PACKET_HEADER_SIZE);
				Serial.write(msg, sizeof(new_encoder_pk_t));
				tx_num++;
				break;
			}
			#else
			case ENCODER_GET_READING:
			{
				header_t headerOut;
				genTimestamp(&headerOut.timestamp_sec, &headerOut.timestamp_usec);

				headerOut.packetnum = tx_num;
				headerOut.cmd = ENCODER_GET_READING;
				headerOut.size = sizeof(new_encoder_pk_t);

				new_encoder_pk_t body;

				noInterrupts();
				int64_t first_left = left_coder_ticks;
				int64_t first_right = right_coder_ticks;
				interrupts();
				_delay_ms(5);
				noInterrupts();
				body.pl = left_coder_ticks;
				body.pr = right_coder_ticks;
				interrupts();

				body.dl = body.pl - first_left;
				body.dr = body.pr - first_right;

				uint8_t* msg = (uint8_t*)&(body);
				Serial.write((uint8_t*)&headerOut, PACKET_HEADER_SIZE);
				Serial.write(msg, sizeof(new_encoder_pk_t));
				tx_num++;
				break;
			}
			#endif
			case ENCODER_RESET_COUNT:
			{
				header_t headerOut;
				genTimestamp(&headerOut.timestamp_sec, &headerOut.timestamp_usec);

				headerOut.packetnum = tx_num;
				headerOut.cmd = ENCODER_RESET_COUNT;
				headerOut.size = 0;

				left_coder_ticks = 0;
				right_coder_ticks = 0;
				//coder_ticks = 0;
				Serial.write((uint8_t*)&headerOut, PACKET_HEADER_SIZE);
				tx_num++;
				break;
			}
			case ARDUINO_HALT_CATCH_FIRE:
			default:
			{
				//HCF();
				//Serial.flush();
				break;
			}
		}//end switch

		if(header.size > 0)
		{
			free(indata);
			indata = NULL;
		}
	}//end for

	return 0;
}//end main

