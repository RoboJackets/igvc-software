/*===============================
========SONAR RANGER 3.04========
===============================*/
#define AVR_ATmega168
#include "Arduino_SD.h"
extern "C"{
	#include "sonar.h"
}

//==========Init Vars====================================
int SDA = PC5;				//Pin ID of I2C SDA
int SCL = PC4;				//Pin ID of I2C SCL
//=======================================================

void setup(){
	Serial.begin(9600);	
}

void loop(){
	readSerial();
	SonDev_Run(millis);
}

