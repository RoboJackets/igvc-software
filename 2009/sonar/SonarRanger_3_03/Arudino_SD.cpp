//TODO:
//	- generalize this to work for all of our code
//	- - pull/push data, read/set one/all/several values, fixed/variable length messages
//	- write SPI library
//	- comment everything

/*
 * This is a template for running arduino code that can be used with the ArudinoInterface class. 
 */

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


/* PIN DEFINITIONS */
//// add your definitions here ////

//// this defines everything that will be sent to the computer
//TODO: make this work for push/pull
//TODO: make this a typedef (must make prototype)

//struct data{};

int heading;
int dbg = 0;
int tx_num,rxn_num;
long global_time_sec;
long global_time_usec;
long arduino_time_millis;
extern SONDEV* sonardevice;
extern int _TOTALDEV;
extern int _STEPFREQ;
extern int _TOTALSTEPS;


void readSerial(void) {
	
	if (Serial.available()) {
		
		//READ HEADER//
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
			
			case 's':{ //this has all the sonar functions --- long ---
				
				while (!Serial.available());
				byte cmd = Serial.read();
				
				switch(cmd){
					
					case SPING_IND:{
						while (!Serial.available());
						byte index = Serial.read();
						sonardevice[index].flags |= (FLAG_PING_SET | FLAG_READ_SET);
						SonDev_Run(delay);
						sendPacket('s',2,(byte*)&sonardevice[index].rdata[0]);
						//sendPacket('s',0,NULL);
						break;
					}
					
					case SREAD_IND:{
						/* THIS IS TAKEN CARE OF IN SPING_IND - remove when cleaning

						int val;
						while (!Serial.available());
						byte index = Serial.read();
						//sonardevice[index].flags |= FLAG_READ_SET;
						//SonDev_Run(delay);
						sendPacket('s',2,sonardevice[index].rdata);
						break;
						*/
					}
					
					case SPING_ALL:{
						byte readdata[16*3+1];
						byte i,devread=0;
						for(i=0;i<_TOTALDEV;i++){
							if(sonardevice[i].flags && FLAG_ACTIVE_SET){
								sonardevice[i].flags |= (FLAG_PING_SET | FLAG_READ_SET);
								readdata[0+devread*3] = i;
								devread ++;
							}
						}

						SonDev_Run(delay);

						for(i=0;i<devread;i++){
							readdata[1+i*3] = sonardevice[i].rdata[0];
							readdata[2+i*3] = sonardevice[i].rdata[1];
						}
						readdata[devread*3] = -1;

						sendPacket('s',devread*3+1,readdata);
						break;	
					}

					case SREAD_ALL:{
						/* THIS IS AWKWARD TO IMPLEMENT -- remove when cleaning
						int devread  = 0;
						int readdata[32];
						byte odata[32];
						for(i=0;i<_TOTALDEV;i++){
							if(sonardevice[i].flags && FLAG_ACTIVE_SET){
								sonardevice[i].flags |= FLAG_READ_SET;
								readdata[devread*2] = i;
								devread ++;
							}
						}
						SonDev_Run(delay);

						//odata = (byte*)calloc(devread*2,sizeof(int)); <-??
						//memcpy(odata,readdata,i*2*sizeof(int)); //<- not being recognized
						sendPacket('s',devread*sizeof(int),odata);						
						break;
						*/
					}

					case SET_GAIN_ALL:{
						while(!Serial.available());
						byte gainval = Serial.read();
						for(i=0;i<_TOTALDEV;i++){
							if(sonardevice[i].flags && FLAG_ACTIVE_SET){
								sonardevice[i].flags |= FLAG_GEDIT_SET;
								sonardevice[i].gain = gainval;
							}
						}
						SonDev_Run(delay);
						sendPacket('s',0,NULL);
						break;
					}

					case SET_GAIN_IND:{
						//dbg = 1;
						while(Serial.available() <= 1);
						byte index = Serial.read();
						byte gainval = Serial.read();
						sonardevice[index].flags |= FLAG_GEDIT_SET;
						sonardevice[index].gain = gainval;
						SonDev_Run(delay);
						sendPacket('s',0,NULL);
						break;
					}

					case GET_GAIN_IND:{
						//dbg = 1;
						while(!Serial.available());
						byte index = Serial.read();
						byte gainval = sonardevice[index].gain;
						sendPacket('s',1,&gainval);
						break;
					}

					case SET_MRANGE_ALL:{
						while(!Serial.available());
						byte mrangeval = Serial.read();
						for(i=0;i<_TOTALDEV;i++){
							if(sonardevice[i].flags && FLAG_ACTIVE_SET){
								sonardevice[i].flags |= FLAG_MREDIT_SET;
								sonardevice[i].mrange = mrangeval;
							}
						}
						SonDev_Run(delay);
						sendPacket('s',0,NULL);
						break;
					}

					case SET_MRANGE_IND:{
						while(Serial.available() <= 1);
						byte index = Serial.read();
						byte mrangeval = Serial.read();
						sonardevice[index].flags |= FLAG_MREDIT_SET;
						sonardevice[index].mrange = mrangeval;
						SonDev_Run(delay);
						sendPacket('s',0,NULL);
						break;
					}

					case GET_MRANGE_IND:{
						//dbg = 1;
						while(!Serial.available());
						byte index = Serial.read();
						byte mrangeval = sonardevice[index].mrange;
						sendPacket('s',1,&mrangeval);
						break;
					}

					case SET_FREQ:{
						while(!Serial.available());
						byte freqval = Serial.read();
						_STEPFREQ = freqval;
						sendPacket('s',0,NULL);
						break;
					}

					case SET_STEP_TOT:{
						while(!Serial.available());
						byte steptotnum = Serial.read();
						_TOTALSTEPS = steptotnum;
						sendPacket('s',0,NULL);	
						break;
					}

					case SET_STEP_IND:{
						while(Serial.available() <= 1);
						byte index = Serial.read();						
						byte stepval = Serial.read();
						sonardevice[index].seqid = stepval;
						sendPacket('s',0,NULL);
						break;
					}

					case SET_ACTIVE_IND:{
						while(Serial.available() <= 1);
						byte index = Serial.read();
						byte activeval = Serial.read();
						(activeval) ? sonardevice[index].flags |= FLAG_ACTIVE_SET : sonardevice[index].flags &= ~FLAG_ACTIVE_SET;
						sendPacket('s',0,NULL);
						break;
					}

					case GET_ACTIVE_IND:{
						while(!Serial.available());
						byte index = Serial.read();
						byte activeval = sonardevice[index].flags & FLAG_ACTIVE_SET;
						sendPacket('s',1,&activeval);
						break;
					}

					case GET_STEP_IND:{
						while(!Serial.available());
						byte index = Serial.read();
						byte stepval = sonardevice[index].seqid;
						sendPacket('s',1,&stepval);
						break;
					}

					case GET_FREQ:{
						sendPacket('s',1,(byte*)&_STEPFREQ);
						break;
					}

					case GET_STEP_TOT:{
						sendPacket('s',1,(byte*)&_TOTALSTEPS);
						break;
					}

					case SET_AUTO_ALL:{ //this causes problems and is hard to implement -- get rid of it
						while (!Serial.available());
						byte state = Serial.read();
						if(state){
							for(i=0;i<_TOTALDEV;i++){
								if(sonardevice[i].flags && FLAG_ACTIVE_SET){
									sonardevice[i].flags |= FLAG_CONT_SET;
									sonardevice[i].flags |= FLAG_PING_SET;
								}
							}
						}else{
							for(i=0;i<_TOTALDEV;i++){
								if(sonardevice[i].flags && FLAG_ACTIVE_SET){
									sonardevice[i].flags &= ~FLAG_CONT_SET;
									sonardevice[i].flags &= ~FLAG_PING_SET;
								}
							}
						}
						SonDev_Run(delay);
						sendPacket('s',0,NULL);

					}
					
					
				}
				break;
			}
			
			default:{
				//error
				break;
			}
		}
	}
}


//TODO: make this use an array
void setVariable(byte num, byte val) {
	switch (num) {
		case MC_SETCLK:
			{			
			unsigned long int sec;
			unsigned long int usec;

			while (Serial.available()<7){}  // TODO: add timeout
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
	//Serial.println(num);  //TODO: check for errors
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
	//serialPrintBytes(leftMotorTick, sizeof(int));  // only 9 bits are used
	//serialPrintBytes(rightMotorTick, sizeof(int));
	serialPrintBytes(&heading, sizeof(double));
}

void serialPrintBytes(void *data, int numBytes) {
	for (int i = 0; i < numBytes; i++) {
		Serial.print(((unsigned char *)data)[i], BYTE);
	}
}

// Fixed point only. Should make scientific notation option.
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
	//TCNT1 is a 16 bit timer/counter ~ pg 121
	time = TCNT1L;//how often does this tick? tests say clkio = F_CPU.
	time |= TCNT1H << 8;
	return(time);
}

void resetTime(){
	TCNT1H = 0;
	TCNT1L = 0;
}

void genTimestamp(long * sec, long * usec)
{
	//*sec = (long)0x00000001;
	//*usec = (long)0x00000001;
	*sec =  global_time_sec + (millis()/1000) - arduino_time_millis/1000;
	*usec = *sec - (global_time_usec + millis()*1000 - arduino_time_millis*1000);
}
