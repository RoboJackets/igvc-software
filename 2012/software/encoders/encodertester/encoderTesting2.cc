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

int main()
{
	setup();
	while(1)
		loop();
	return 0;
}
void setup()
{
	Serial.begin(57600);
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
	Serial.println(countA);
	Serial.println(countB);
	Serial.println(0);//lame way to seperate pairs of value due to uncertainty as to how method works
