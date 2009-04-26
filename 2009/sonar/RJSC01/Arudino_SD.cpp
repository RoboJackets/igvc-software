
#include "Arduino_SD.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/signal.h>
#include <avr/delay.h>
#include <stdarg.h>
#include <wiring.h>
#include "HardwareSerial.h"
extern "C"{
#include "sonar.h"
}

//=========Init Vars==================
int heading;			//Heading 
int tx_num,rxn_num;		//Current Tx/Rx Packet Counts
long global_time_sec;		//Global Time Var
long global_time_usec;		//Global Time Var
long arduino_time_millis;	//Arduino Time Var
extern int _TOTALDEV;		//_Total Number of Devices_
extern int _STEPFREQ;		//_Step Frequency_
extern int _TOTALSTEPS;		//_Total Steps_
extern byte proc_buff[5];	//SonDev Processor Buffer
//====================================


void readSerial(void) {
	if (Serial.available()) {
		int i;
		header_t headerIn;
		byte* hdrptr = (byte*)&headerIn;

		while (Serial.available()<PACKET_HEADER_SIZE);

		for(i=0;i<PACKET_HEADER_SIZE;i++){
			*(hdrptr+i) = Serial.read(); //take care of this
		}

		switch(headerIn.cmd){ //TODO: make these char --> defines
			
			case 'w':{
	
				while (Serial.available()<2){}  // TODO: add timeout
				byte variableNumber = Serial.read();
				byte variableValue = Serial.read();
				setVariable(variableNumber, variableValue);
				sendPacket(ARDUINO_SETVAR_RESP,0,NULL);
				break;
			}
			
			case 'i':{
				byte msg = 's';
				sendPacket(ARDUINO_ID_RESP,1,&msg);
				break;
			}
			
			case 's':{
				int i;
				while (Serial.available()<headerIn.size);
				for(i=0;i<headerIn.size;i++){
					proc_buff[i] = Serial.read();
				}
				SonDev_Process(proc_buff,sendPacket);
				break;
			}
			
			default:{
				//error 
				break;
			}
		}
	}
}

void setVariable(byte num, byte val) {
	switch (num) {
		case MC_SETCLK:
			{			
			unsigned long int sec;
			unsigned long int usec;

			while (Serial.available()<7){}
			byte * bptr = (byte *)&sec;
			bptr[0] = val;
			bptr[1] = Serial.read();
			bptr[2] = Serial.read();
			bptr[3] = Serial.read();

			bptr = (byte *)&usec;
			bptr[0] = Serial.read();
			bptr[1] = Serial.read();
			bptr[2] = Serial.read();
			bptr[3] = Serial.read();

			global_time_sec = sec;
			global_time_usec = usec;
			arduino_time_millis = millis();
			break;
			}
		default:
			//error
			break;
	}
}

void sendPacket(char cmd, int dataSize, byte* data) {
	header_t headerOut;
	genTimestamp(&headerOut.timestamp_sec, &headerOut.timestamp_usec);
	headerOut.packetnum = tx_num;
	headerOut.cmd = cmd;
	headerOut.size = dataSize;
	serialPrintBytes(&headerOut, PACKET_HEADER_SIZE);
	serialPrintBytes(data,dataSize);
	tx_num++;
}

void sendStatus() {
	serialPrintBytes(&heading, sizeof(double));
}

void serialPrintBytes(void *data, int numBytes) {
	for (int i = 0; i < numBytes; i++) {
		Serial.print(((unsigned char *)data)[i], BYTE);
	}
}

void serialPrintDouble(double data, int precision) {
	Serial.print((int)data);
	Serial.print(".");
	for(int i = 1; i < (precision+1); i++) {
		data = (data - (double)((int)data)) * 10;
		Serial.print((int)data);
	}
}

unsigned int getTime(){
	unsigned int time;
	time = TCNT1L;
	time |= TCNT1H << 8;
	return(time);
}

void resetTime(){
	TCNT1H = 0;
	TCNT1L = 0;
}

void genTimestamp(long * sec, long * usec)
{
	*sec =  global_time_sec + (millis()/1000) - arduino_time_millis/1000;
	*usec = *sec - (global_time_usec + millis()*1000 - arduino_time_millis*1000);
}
