#include "WProgram.h"

//left to right is positive
//A is bit 2
//B is low bit
const int coderstates[] = {0, 1, 3, 2};

#define PIN_A 2
#define PIN_B 3

extern "C" void __cxa_pure_virtual()
{
	for(;;)
	{

	}
}

void forward_quad_step()
{
	static int pos = 0;

	unsigned int a = (coderstates[pos] & 0x02);
	unsigned int b = (coderstates[pos] & 0x01);

	PORTD = (unsigned(a|b) << 2);

	pos++;
	if(pos > 3)
	{
		pos = 0;
	}
}

void backward_quad_step()
{
	static int pos = 0;

	unsigned int a = (coderstates[pos] & 0x02);
	unsigned int b = (coderstates[pos] & 0x01);

	PORTD = (unsigned(a|b) << 2);

	pos--;
	if(pos < 0)
	{
		pos = 3;
	}
}

int main()
{
	init();

	Serial.begin(57600);

	pinMode(PIN_A, OUTPUT);
	pinMode(PIN_B, OUTPUT);

	digitalWrite(PIN_A, LOW);
	digitalWrite(PIN_B, LOW);

	//PORTD &= (0xF3);//0000 1100 - 1111 0011

	int step = 0;
	while(step < 10000)
	{
		forward_quad_step();
		step++;

		//backward_quad_step();
		//step--;

		//delay(1000);// 1 Hz
		//delayMicroseconds(1000);// 1 kHz
		//delayMicroseconds(100);// ~10 kHz ?
		delayMicroseconds(10);// ~100 kHz ?
	}

	for(;;)
	{
		Serial.print("Step ");
		Serial.println(step, DEC);
		delay(1000);// 1 Hz
	}

}
