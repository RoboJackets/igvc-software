//#include "pinDefs.h"
int rightDirectionPin = 2;
int rightSpeedPin = 3;
int rightDisablePin = 4;
int leftDirectionPin = 5;
int leftSpeedPin = 6;
int leftDisablePin = 7;

int  main(void)
{
	//init();
	
	setup();

		for (;;)
			loop();

		return 0;
}

void setup() {
  // initialize the digital pin as an output.
  // Pin 13 has an LED connected on most Arduino boards:
			pinMode(leftSpeedPin, OUTPUT);
			pinMode(rightDirectionPin, OUTPUT);
			pinMode(leftDirectionPin, OUTPUT);
			pinMode(rightSpeedPin, OUTPUT);
			setPWMFreq();
			analogWrite(leftSpeedPin,0);
			analogWrite(rightSpeedPin,0);
			pinMode(rightDisablePin, OUTPUT);
			pinMode(leftDisablePin, OUTPUT);
			digitalWrite(rightDisablePin, LOW);
			digitalWrite(leftDisablePin, LOW);
			digitalWrite(leftDirectionPin, HIGH);
			digitalWrite(rightDirectionPin, HIGH);

			// setPWMFreq(); --- Change while testing
}

void loop() {

	for (int i=0; i<200;i+=2)
	{

		analogWrite(leftSpeedPin,200);
		analogWrite(rightSpeedPin,i);
		delay(100);

	}
      for (int i=200; i>0;i-=2)
	{

		analogWrite(leftSpeedPin,255-i);
		analogWrite(rightSpeedPin,i);
		delay(100);

	}
}

void setPWMFreq()
{
	TCCR2B = TCCR2B & 0xF8 | 0x03;//pin 11/3 @ 976.5625Hz
	TCCR0B = TCCR0B & 0xF8 | 0x03;//pins 5/6 @ 976.5625Hz
	//TCCR2B = (TCCR2B & 0xF8) | 0x02;//pin 11/3 @ 2x976.5625Hz
	//TCCR0B = (TCCR0B & 0xF8) | 0x02;//pins 5/6 @ 2x976.5625Hz
}






