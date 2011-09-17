#include "WProgram.h"
#include "HardwareSerial.h"

#include "serial_comm_helper.hpp"

#include "DataPacketStructs.hpp"
#include "ArduinoCmds.hpp"
#include "packet_handle.hpp"
#include "wheel_encoders.hpp"
#include "current_sensors.hpp"
#include "motorPWM.hpp"
#include "analogjoystick.hpp"
#include "magnetometer.hpp"

#include "common_defines.hpp"

//#include "profile.hpp"
extern "C" void __cxa_pure_virtual()
{
	for(;;)
	{

	}
}
#if 0
void HCF()
{
	pinMode(13, OUTPUT);
	for(;;)
	{
		digitalWrite(13, LOW);
		Serial.println("HCF: LOW");
		delay(1000);
		digitalWrite(13, HIGH);
		Serial.println("HCF: HIGH");
		delay(1000);
	}
}
#endif
int main()
{	
	init();
	Serial.begin(SERIAL_BAUD);

	unsigned long tx_num = 0;
	unsigned long rx_num = 0;
	bool analogJoyOn = false;
	unsigned long lastsettime = 0;
	//setup
	//pinMode(leftCurrentADCPin, INPUT);
	//pinMode(rightCurrentADCPin, INPUT);

	pinMode(leftSpeedPin, OUTPUT);
	pinMode(rightDirectionPin, OUTPUT);
	pinMode(leftDirectionPin, OUTPUT);
	pinMode(rightSpeedPin, OUTPUT);
	setPWMFreq();
	setRightMotorDutyCycle(MC_MOTOR_FORWARD, 0);
	setLeftMotorDutyCycle(MC_MOTOR_FORWARD, 0);

	pinMode(rightDisablePin, OUTPUT);
	pinMode(leftDisablePin, OUTPUT);
	digitalWrite(rightDisablePin, LOW);
	digitalWrite(leftDisablePin, LOW);

	pinMode(LIGHT_PIN_LB, OUTPUT);
	digitalWrite(LIGHT_PIN_LB, HIGH);

	setupJoystick();
	//end setup

	//test
#if 0
	for(;;)
{
		setRightMotorDutyCycle(MC_MOTOR_REVERSE, 200);
		//setLeftMotorDutyCycle(MC_MOTOR_FORWARD, 200);
		setLeftMotorDutyCycle(MC_MOTOR_REVERSE, 20);
		for(;;)
		{
			delay(50);
		}
}
#endif
	//test

	//arduinoProfile aP(13, 12);

	//aP.print("startup\n\r");
	unsigned long joystickpollt0 = millis();
	for(;;)
	{

		if((millis() - lastsettime) > 2e3)
		{
			if(!analogJoyOn)
			{
				setRightMotorDutyCycle(MC_MOTOR_FORWARD, 0);
				setLeftMotorDutyCycle(MC_MOTOR_FORWARD, 0);
			}
		}

		if(digitalRead(joystickEnable) == LOW)
		{
			analogJoyOn = true;
			//Serial.println("joy on");
			if((millis() - joystickpollt0) > 10)
			{
				//Serial.println("joy set");

				int x,y;
				getJoystickReading(&x, &y);

				//Serial.print("x: ");
				//Serial.print(x, DEC);
				//Serial.print("\ty: ");
				//Serial.println(y, DEC);
				joystickSetMotors();
				joystickpollt0 = millis();	
			}
		}
		else
		{
			analogJoyOn = false;
		}
		
		if(!(Serial.available() > 1))
		{
			continue;
		}

		header_t header;
		byte* indata = NULL;
		//byte indata[20];
		//memset(indata, 0, 20);
		if(!serialReadBytesTimeout(PACKET_HEADER_SIZE, (byte*)&header))
		{
			continue;
		}

		if(header.size > 0)
		{
			indata = (byte*)malloc(header.size);
			if(!serialReadBytesTimeout(header.size, indata))
			{
				free(indata);
				indata = NULL;
				/*
				for(;;)
				{
					Serial.print("header.size: ");
					Serial.println(header.size, DEC);

					Serial.print("sizeof(header): ");
					Serial.println(sizeof(header_t), DEC);

					byte* dptr = (byte*)&header;
					for(size_t i = 0; i < sizeof(header_t); i++)
					{
						Serial.print("i[");
						Serial.print(i);
						Serial.print("]: ");
						Serial.println(dptr[i], DEC);
					}
					delay(1000);
				}
				*/
				//HCF();
				continue;
			}
		}

		//Serial.println("Header cmd: ");
		//Serial.println(header.cmd, DEC);
		//continue;
		//aP.profileStartSec();
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
			case ARDUINO_GET_ID:
			{
				header_t headerOut;
				genTimestamp(&headerOut.timestamp_sec, &headerOut.timestamp_usec);

				headerOut.packetnum = tx_num;
				headerOut.cmd = ARDUINO_GET_ID;
				headerOut.size = 1;
				//char msg = OSMC_IF_BOARD;
				//char msg = OSMC_IF_FOR_BOARD;
				char msg = OSMC_IF_AFT_BOARD;

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
 			case MC_SET_RL_DUTY_CYCLE:
			{
				//Serial.println("Set R Speed");
				speed_set_t dcp;

				memcpy(&dcp, indata, sizeof(speed_set_t));

				//if the analog joystick is enabled, accept the serial command and resp success, but do not take the input
				if(!analogJoyOn)
				{
					setRightMotorDutyCycle(dcp.rightDir, dcp.sr);
					setLeftMotorDutyCycle(dcp.leftDir, dcp.sl);
				}

				header_t headerOut;
				genTimestamp(&headerOut.timestamp_sec, &headerOut.timestamp_usec);
				headerOut.packetnum = tx_num;
				headerOut.cmd = MC_SET_RL_DUTY_CYCLE;
				headerOut.size = 0;
		
				serialPrintBytes(&headerOut, PACKET_HEADER_SIZE);
				tx_num++;

				lastsettime = millis();
				break;
			}
			case MC_GET_JOYSTICK:
			{
				joystick_reply_t data;
				getJoystickReading(&(data.joy_x), &(data.joy_y));

				header_t headerOut;
				genTimestamp(&headerOut.timestamp_sec, &headerOut.timestamp_usec);
				headerOut.packetnum = tx_num;
				headerOut.cmd = MC_GET_JOYSTICK;
				headerOut.size = sizeof(joystick_reply_t);
		
				serialPrintBytes(&headerOut, PACKET_HEADER_SIZE);
				serialPrintBytes(&data, headerOut.size);
				tx_num++;
				break;
			}
			case MAG_GET_MAGDATA:
			{
				magnetometer_pk_t heading;
				header_t headerOut;
				genTimestamp(&headerOut.timestamp_sec, &headerOut.timestamp_usec);
				headerOut.packetnum = tx_num; 
				headerOut.cmd = MAG_GET_MAGDATA;

				// Defined in new file magnetometer.cc
				heading = getHeading();

				headerOut.size = sizeof(magnetometer_pk_t);
		
				serialPrintBytes(&headerOut, PACKET_HEADER_SIZE);
				serialPrintBytes(&heading, headerOut.size);
				tx_num++;
				break;
			}
			case MC_SET_LIGHT:
			{
				switch(indata[0])
				{
					case MC_LIGHT_STEADY:
					{
						digitalWrite(LIGHT_PIN_LB, HIGH);
						break;					
					}
					case MC_LIGHT_PULSING:
					{	
						digitalWrite(LIGHT_PIN_LB, LOW);
						break;
					}
				}
			}
			case ARDUINO_HALT_CATCH_FIRE:
			default:
			{
				//HCF();
				break;
			}
		}//end switch

		if(header.size > 0)
		{
			free(indata);
			indata = NULL;
		}
		//aP.profileEndSec("Switch/Case");
	}//end for

	return 0;
}//end main

