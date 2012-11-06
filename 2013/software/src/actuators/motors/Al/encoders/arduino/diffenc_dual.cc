#include "diffenc.hpp"
#include "pindefs.hpp"

//this is currently setup for 4 interupts on 4 data pins
//could be done using two interupts and 4 additional datapins -- do the or for left A/B event in hardware, or buy a bigger atmega / arduino mega

volatile int64_t left_coder_ticks;
volatile int64_t right_coder_ticks;

enum trigger{TRIG_LEFT, TRIG_RIGHT};

void leftenc_event()
{ 
	// Left wheel CW means it's going foward
	if (digitalRead(left_encoder_pin_B) == HIGH)
		left_coder_ticks++;
	else
		left_coder_ticks--;
	//encoder_logger(TRIG_LEFT);
}

void rightenc_event()
{
	// Right wheel CCW means it'g going backward
	if (digitalRead(right_encoder_pin_B) == LOW)
		right_coder_ticks++;
	else
		right_coder_ticks--;
	//encoder_logger(TRIG_RIGHT);
}

void encoder_logger(int trigger)
{
	unsigned int currstate = 0;
	static unsigned int laststate = 0;

	currstate = (PIND & 0x0C);//get bits for pin 2&3

	if(TRIG_LEFT)
	{
		incr_or_dec(currstate, laststate, &left_coder_ticks);
	}
	else
	{
		incr_or_dec(currstate, laststate, &right_coder_ticks);
	}
	laststate = currstate;
}

//decode quadrature encoder input as a func of the current state and last state
//the state is defined as x in {0x00, 0x4, 0x8, 0xC}, with ch A being the top bit
// 0: 0x00
// A: 0x04
// B: 0x08
// A&B: 0x0C
void incr_or_dec(const int currstate, const int laststate, volatile int64_t* counter)
{
	/*
	http://www.mcmanis.com/chuck/robotics/projects/lab-x3/quadratrak.html
	http://en.wikipedia.org/wiki/Rotary_encoder

	 clockwise ->
	00 01 11 10 00
	   <- ccw

	need to verify cw/ccw dir on motor
	*/

	switch(laststate)
	{
		case 0: // 00
			switch(currstate)
			{
				case 0: // 00 -> 00
					return;
				case 0x4: // 00 -> 01
					(*counter)++;
					return;
				case 0x8: // 00 -> 10
					(*counter)--;
					return;
				case 0xc: // 00 -> 11
					return;
			}
		case 0x04: // 01
			switch(currstate)
			{
				case 0: // 01 -> 00
					(*counter)--;
					return;
				case 0x4: // 01 -> 01
					return;
				case 0x8: // 01 -> 10
					return;
				case 0xc: // 01 -> 11
					(*counter)++;
					return;
			}
		case 0x08: // 10
			switch(currstate)
			{
				case 0: // 10 -> 00
					(*counter)++;
					return;
				case 0x4: // 10 -> 01
					return;
				case 0x8: // 10 -> 10
					return;
				case 0xc: // 10 -> 11
					(*counter)--;
					return;
			}
		case 0x0C: // 11
			switch(currstate)
			{
				case 0: // 11 -> 00
					return;
				case 0x4: // 11 -> 01
					(*counter)--;
					return;
				case 0x8: // 11 -> 10
					(*counter)++;
					return;
				case 0xc: // 11 -> 11
					return;
			}
	}
	
}
