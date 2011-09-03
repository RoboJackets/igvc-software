// Tests Arduino boards for shorted pins and non-functioning PWN pins
// Kenneth Marino

#include "WProgram.h"
void flashLed();
void setAllout();
void setAllin();
void allAnOut();
static const int time = 5000;
const int anpin0 = 14;
const int anpin1 = 15;
const int anpin2 = 16;
const int anpin3 = 17;
const int anpin4 = 18;
const int anpin5 = 19;


int main()
{
	init();
	
	setAllout();

	for(;;)
	{
		flashLed();		
	}
	
	//setAllout();
	//for(;;)
	//{
	//	allAnOut();
	//}	

}

void flashLed()
{
	
	
	for(int x = 0; x<20; x++)
	{
		digitalWrite(x, HIGH);	
	}	
	
	delay(time);

	for(int x = 0; x<=19; x++)
	{
		digitalWrite(x, LOW);	
	}
	
	delay(time);
}

void setAllout()
{
	for(int x = 0; x<=19; x++)	
		{
			pinMode(x, OUTPUT);	
		}
}

void setAllin()
{	for(int x = 0; x<=19; x++)	
		{
			pinMode(x, INPUT);	
		}
}
	
void allAnOut()
{
	//for(int dutc= 0; dutc <= 255; dutc++)
	int dutc = 127;
	analogWrite(3, dutc);
	analogWrite(5, dutc);
	analogWrite(6, dutc);
	analogWrite(9, dutc);
	analogWrite(10, dutc);
	analogWrite(11, dutc);
}

