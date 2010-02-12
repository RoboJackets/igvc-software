
#include "WProgram.h"

#include "diffenc.hpp"
#include "pindefs.hpp"

extern "C" void __cxa_pure_virtual()
{
	for(;;)
	{

	}
}

int main()
{

	init();

	Serial.begin(57600);

	pinMode(left_encoder_pin_A, INPUT);
	pinMode(left_encoder_pin_B, INPUT);

	pinMode(right_encoder_pin_A, INPUT);
	pinMode(right_encoder_pin_B, INPUT);

	attachInterrupt(0, leftenc_event_A, CHANGE);
	attachInterrupt(1, leftenc_event_B, CHANGE);

	//attachInterrupt(2, rightenc_event_A, CHANGE);
	//attachInterrupt(3, rightenc_event_B, CHANGE);

	left_ticks = 0;
	right_ticks = 0;

	for(;;)
	{

		int64_t left_first = left_ticks;
		int64_t right_first = right_ticks;

		delay(20);

		int64_t left_second = left_ticks;
		int64_t right_second = right_ticks;

		int64_t dl = left_second - left_first;
		int64_t dr = right_second - right_first;

		float lv = float(dl) / float(.1);
		float rv = float(dr) / float(.1);

		Serial.print("l: ");
		Serial.print(left_ticks, DEC);
		Serial.print("\tr: ");
		Serial.print(right_ticks, DEC);
		Serial.print("\tlv: ");
		Serial.print(lv, DEC);
		Serial.print("\trv: ");
		Serial.print(rv, DEC);

		delay(20);
	}


	return 0;
}
