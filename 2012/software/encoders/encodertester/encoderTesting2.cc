#include "WProgram.h"

#define encoderPinA 4
#define encoderPinB 6
#define encoderOutputI 5

int counter=0;
int stateA;
int stateB;
int lastStateA=LOW;
int lastStateB=LOW;

void setup()
{
/*	serial.Begin(57600);
	pinMode(encoderPinA,INPUT);
	pinMode(encoderPinB,INPUT); */
}

void loop()
{
	/*stateA=digitalRead(encoderPinA);
	stateB=digitalRead(encoderPinB);
	if(stateA=!lastStateA && encoderPinB!=lastStateB)
	{
		counter++;
		lastStateA=stateA;
		lastStateB=stateB;
	}
	Serial.println(analogValue,counter);*/
}
