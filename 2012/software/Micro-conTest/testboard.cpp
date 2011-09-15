#include "WProgram.h"

const int analogPin1 = 3;      // Yay pins
const int analogPin2 = 5;   
const int analogPin3 = 6;
const int analogPin4 = 9;
const int analogPin5 = 10;
const int analogPin6 = 11;
const int digitalPin1 = 1;
const int digitalPin2 = 2;
const int digitalPin3 = 4;
const int digitalPin4 = 7;
const int digitalPin5 = 8;
const int digitalPin6 = 12;
const int digitalPin7 = 13;

int main()
{
	setup();
	for(;;)
	{
		loop();
	}
}


void setup()
{
	pinMode(digitalPin1, OUTPUT);   // output
	pinMode(digitalPin2, OUTPUT); 
	pinMode(digitalPin3, OUTPUT); 
	pinMode(digitalPin4, OUTPUT); 
	pinMode(digitalPin5, OUTPUT); 
	pinMode(digitalPin6, OUTPUT); 
	pinMode(digitalPin7, OUTPUT); 
}

void loop()
{
	digitalWrite(digitalPin1, HIGH); //Digital output
	digitalWrite(digitalPin2, HIGH); 
	digitalWrite(digitalPin3, HIGH); 
	digitalWrite(digitalPin4, HIGH); 
	digitalWrite(digitalPin5, HIGH); 
	digitalWrite(digitalPin6, HIGH); 
	digitalWrite(digitalPin7, HIGH); 
	analogWrite(analogPin1, 192); // PWM output
	analogWrite(analogPin2, 192);
	analogWrite(analogPin3, 192);
	analogWrite(analogPin4, 192);
	analogWrite(analogPin5, 192);
	analogWrite(analogPin6, 192);
	delay(10000);
	digitalWrite(digitalPin1, LOW); //yay silence
	digitalWrite(digitalPin2, LOW); 
	digitalWrite(digitalPin3, LOW); 
	digitalWrite(digitalPin4, LOW); 
	digitalWrite(digitalPin5, LOW); 
	digitalWrite(digitalPin6, LOW); 
	digitalWrite(digitalPin7, LOW); 
	analogWrite(analogPin1, 64);
	analogWrite(analogPin2, 64);
	analogWrite(analogPin3, 64);
	analogWrite(analogPin4, 64);
	analogWrite(analogPin5, 64);
	analogWrite(analogPin6, 64);
	delay(10000);
}

