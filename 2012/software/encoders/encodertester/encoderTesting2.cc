#include "WProgram.h"

#define encoderPinA 4
#define encoderPinB 6
#define encoderOutputI 5

int countA;
int countB;
int stateA;
int stateB;
int lastStateA=LOW;
int lastStateB=LOW;

void setup()
{
	pinMode(encoderPinA,INPUT);
	pinMode(encoderPinB,INPUT);
	stateA=lastStateA=digitalRead(encoderPinA);
	stateB=lastStateB=digitalRead(encoderPinB);
}

void loop()
{
	stateA=digitalRead(encoderPinA);
	stateB=digitalRead(encoderPinB);
	if(stateA==HIGH && lastStateA!=stateA)
	{
		countA++;
	}
	if(stateB==HIGH && lastStateB!=stateB)
	{
		countB++;
	}
	lastStateA=stateA;
	lastStateB=stateB;
