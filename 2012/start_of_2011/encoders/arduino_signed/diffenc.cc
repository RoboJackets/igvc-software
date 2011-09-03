#include "diffenc.hpp"
#include "pindefs.hpp"

//this is currently setup for 4 interupts on 4 data pins
//could be done using two interupts and 4 additional datapins -- do the or for left A/B event in hardware, or buy a bigger atmega / arduino mega

volatile int64_t coder_ticks;

void encoder_logger()
{
	unsigned int currstate = 0;
	static unsigned int laststate = 0;

	//currstate = ((PIND & 0x0C) >> 2) & (~0xFC);//get bits for pin 2&3, shift to low, mask. Can remove shift by adjusting code below, then just modify mask
	currstate = (PIND & 0x0C);//get bits for pin 2&3

	//incr_or_dec(currstate, laststate, &left_ticks);
	incr_or_dec_fast(currstate, laststate);

	laststate = currstate;
}

//decode quadrature encoder input as a func of the current state and last state
//the state is defined as x in {0x00, 0x4, 0x8, 0xC}, with ch A being the top bit
// 0: 0x00
// A: 0x04
// B: 0x08
// A&B: 0x0C
void incr_or_dec_fast(const int currstate, const int laststate)
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
				case 0x04: // 00 -> 01
					coder_ticks++;
					return;
				case 0x08: // 00 -> 10
					coder_ticks--;
					return;
				case 0x0C: // 00 -> 11
					return;
			}
		case 0x04: // 01
			switch(currstate)
			{
				case 0: // 01 -> 00
					coder_ticks--;
					return;
				case 0x04: // 01 -> 01
					return;
				case 0x08: // 01 -> 10
					return;
				case 0x0C: // 01 -> 11
					coder_ticks++;
					return;
			}
		case 0x08: // 10
			switch(currstate)
			{
				case 0: // 10 -> 00
					coder_ticks++;
					return;
				case 0x04: // 10 -> 01
					return;
				case 0x08: // 10 -> 10
					return;
				case 0x0C: // 10 -> 11
					coder_ticks--;
					return;
			}
		case 0x0C: // 11
			switch(currstate)
			{
				case 0: // 11 -> 00
					return;
				case 0x04: // 11 -> 01
					coder_ticks--;
					return;
				case 0x08: // 11 -> 10
					coder_ticks++;
					return;
				case 0x0C: // 11 -> 11
					return;
			}
	}
	
}

//decode quadrature encoder input as a func of the current state and last state
//the state is defined as x in [0,3], with ch A being the top bit
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
				case 1: // 00 -> 01
					(*counter)++;
					return;
				case 2: // 00 -> 10
					(*counter)--;
					return;
				case 3: // 00 -> 11
					return;
			}
		case 1: // 01
			switch(currstate)
			{
				case 0: // 01 -> 00
					(*counter)--;
					return;
				case 1: // 01 -> 01
					return;
				case 2: // 01 -> 10
					return;
				case 3: // 01 -> 11
					(*counter)++;
					return;
			}
		case 2: // 10
			switch(currstate)
			{
				case 0: // 10 -> 00
					(*counter)++;
					return;
				case 1: // 10 -> 01
					return;
				case 2: // 10 -> 10
					return;
				case 3: // 10 -> 11
					(*counter)--;
					return;
			}
		case 3: // 11
			switch(currstate)
			{
				case 0: // 11 -> 00
					return;
				case 1: // 11 -> 01
					(*counter)--;
					return;
				case 2: // 11 -> 10
					(*counter)++;
					return;
				case 3: // 11 -> 11
					return;
			}
	}
	
}
