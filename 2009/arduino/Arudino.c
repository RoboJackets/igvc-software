//TODO:
//	- generalize this to work for all of our code
//	- - pull/push data, read/set one/all/several values, fixed/variable length messages
//	- write SPI library
//	- comment everything

/*
 * This is a template for running arduino code that can be used with the ArudinoInterface class. 
 */
#define ATMEGA168

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/signal.h>
#include <avr/delay.h>
//#include <stdio.h>
#include <stdarg.h>

/* PIN DEFINITIONS */
//// add your definitions here ////

//// this defines everything that will be sent to the computer
//TODO: make this work for push/pull
//TODO: make this a typedef (must make prototype)
struct data{
};

/* VARIABLES */
/* SERIAL */
int incomingByte = 0;

void setup(void) {
	/* open the serial port */
	Serial.begin(9600);
	
	/* set the pin modes */
}

void loop(void) {

	// YOUR FUNCTIONS
	readSerial();
}

void readSerial(void) {
	if (Serial.available() > 0) {
		incomingByte = Serial.read();
		if (incomingByte == 'r') {
			sendStatus();
		} else if (incomingByte == 'w') {
			while (Serial.available()<2){}  // TODO: add timeout
			int variableNumber = Serial.read();
			int variableValue = Serial.read();
			setVariable(variableNumber, variableValue);
		} else if (incomingByte == 'i') {
			Serial.print("e");
		} else {
			//error
		}
	}
}

//TODO: make this use an array
void setVariable(int num, int val) {
	switch (num) {
		case 0:
			heading=val;
			break;
		default:
			//error
			break;
	}
	Serial.println(num);  //TODO: check for errors
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

