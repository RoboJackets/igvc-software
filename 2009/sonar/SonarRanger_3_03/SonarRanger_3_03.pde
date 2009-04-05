/*===============================
========SONAR RANGER 3.00========
===============================*/

//External Libraries
#define AVR_ATmega168
#include "Arduino_SD.h"
extern "C" {
#include "sonar.h"
}



int SDA = PC5;				//Pin ID of SDA
int SCL = PC4;				//Pin ID of SCL
int STI = 12;
extern int dbg;
extern SONDEV* sonardevice;

//
void setup(){
	pinMode(STI,OUTPUT);digitalWrite(STI,LOW); //debug
	Serial.begin(9600);	
	SonDev_Start(10,3,2000,14);
}


void loop(){

	readSerial();
	//SonDev_Run(delay);	
	digitalWrite(STI,dbg); //debug
	
}


/*
void loop(){
	

	if (Serial.available() >= 1){
	
		if(Serial.read() == DBG_C){
			sonardevice[1].gain = 12;
			sonardevice[1].flags |= (FLAG_MREDIT_SET);//(FLAG_PING_SET | FLAG_READ_SET);
			sonardevice[0].flags |= (FLAG_PING_SET | FLAG_READ_SET);
			Serial.print(':');
			serialPrintBytes(&sonardevice[0].rangedata,2);
		}
		
	}
	digitalWrite(STI,dbg); //debug
	
	SonDev_Run(delay); //for some reason, a direct call to delay freezes arduino

}*/

