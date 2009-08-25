#include "WProgram.h"

#include "serial_comm_helper.hpp"

#include "DataPacketStructs.hpp"
#include "ArduinoCmds.hpp"
#include "packet_handle.hpp"
#include "wheel_encoders.hpp"
#include "current_sensors.hpp"
#include "motorPWM.hpp"

extern "C" void __cxa_pure_virtual()
{
	for(;;)
	{

	}
}

void HCF()
{
	pinMode(13, OUTPUT);
	for(;;)
	{
		digitalWrite(13, LOW);
		Serial.println("LOW");
		delay(1000);
		digitalWrite(13, HIGH);
		Serial.println("HIGH");
		delay(1000);
	}
}

int main()
{
	
	init();
	Serial.begin(9600);

	unsigned long tx_num = 0;
	unsigned long rx_num = 0;

	//start setting pin modes
	pinMode(leftCurrentADCPin, INPUT);
	pinMode(rightCurrentADCPin, INPUT);
	pinMode(leftSpeedPin, OUTPUT);
	pinMode(rightSpeedPin, OUTPUT);
	//end setting pin modes


	for(;;)
	{

		while(Serial.available() < 1)
		{
			//spin
		}

		header_t header;
		byte* indata = NULL;
		if(!serialReadBytesTimeout(PACKET_HEADER_SIZE, (byte*)&header))
		{
			continue;
		}

		if(header.size > 0)
		{
			indata = (byte*)malloc(header.size);
			serialReadBytesTimeout(header.size, indata);
		}

		//Serial.println("Header cmd: ");
		//Serial.println(header.cmd, DEC);
		//continue;

		switch(header.cmd)
		{
			case ARDUINO_RESET:
			{
				Serial.println("Reset");
				break;
			}
			case ARDUINO_SET_CLOCK:
			{
				Serial.println("Set Clock");
				break;
			}
			case ARDUINO_GET_ID:
			{
				header_t headerOut;
				genTimestamp(&headerOut.timestamp_sec, &headerOut.timestamp_usec);

				headerOut.packetnum = tx_num;
				headerOut.cmd = ARDUINO_GET_ID;
				headerOut.size = 1;
				char msg = OSMC_IF_BOARD;
				//serialPrintBytes(&headerOut, PACKET_HEADER_SIZE);
				Serial.write((uint8_t*)&headerOut, PACKET_HEADER_SIZE);
				Serial.print(msg);
				//savePacket(headerOut, (unsigned char*)&msg);
				tx_num++;
				break;
			}
			case MC_GET_ENCODER_TICK:
			{
				header_t headerOut;
				genTimestamp(&headerOut.timestamp_sec, &headerOut.timestamp_usec);
				headerOut.packetnum = tx_num;
				headerOut.cmd = MC_GET_ENCODER_TICK;

				reply_dtick_t msg = getEncoderStatus();
				headerOut.size = sizeof(reply_dtick_t);
		
				serialPrintBytes(&headerOut, PACKET_HEADER_SIZE);
				serialPrintBytes(&msg, headerOut.size);
				tx_num++;
				break;
			}
			case MC_GET_RL_CURR_VAL:
			{
				header_t headerOut;
				genTimestamp(&headerOut.timestamp_sec, &headerOut.timestamp_usec);
				headerOut.packetnum = tx_num;
				headerOut.cmd = MC_GET_RL_CURR_VAL;

				current_reply_t msg = getBothCurrentADCVal();
				headerOut.size = sizeof(current_reply_t);

				serialPrintBytes(&headerOut, PACKET_HEADER_SIZE);
				serialPrintBytes(&msg, headerOut.size);
				tx_num++;
				break;
			}
			case MC_GET_R_CURR_VAL:
			{
				Serial.println("Get Current");
				break;
			}
			case MC_GET_L_CURR_VAL:
			{
				Serial.println("Get Current");
				break;
			}
 			case MC_SET_R_SPEED:
			{
				Serial.println("Set R Speed");
				break;
			}
			case MC_SET_L_SPEED:
			{
				Serial.println("Set L Speed");
				break;
			}
 			case MC_SET_RL_DUTY_CYCLE:
			{
				//Serial.println("Set R Speed");

				speed_set_t dcp;

				memcpy(&dcp, indata, sizeof(speed_set_t));

				setRightMotorDutyCycle(dcp.sr);
				setLeftMotorDutyCycle(dcp.sl);

				header_t headerOut;
				genTimestamp(&headerOut.timestamp_sec, &headerOut.timestamp_usec);
				headerOut.packetnum = tx_num;
				headerOut.cmd = MC_SET_RL_DUTY_CYCLE;
				headerOut.size = 0;
		
				serialPrintBytes(&headerOut, PACKET_HEADER_SIZE);
				tx_num++;
				break;
			}
			default:
			{
				HCF();
				break;
			}
		}
		if(header.size > 0)
		{
			free(indata);
			indata = NULL;
		}
	}

	return 0;
}
