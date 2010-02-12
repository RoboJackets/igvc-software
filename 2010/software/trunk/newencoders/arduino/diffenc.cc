#include "diffenc.hpp"
#include "pindefs.hpp"

//this is currently setup for 4 interupts on 4 data pins
//could be done using two interupts and 4 additional datapins -- do the or for left A/B event in hardware, or buy a bigger atmega / arduino mega

volatile int64_t left_ticks;
volatile int64_t right_ticks;

void leftenc_event_A()
{
	encoder_logger(ENCODEREVENT_leftA);
}

void leftenc_event_B()
{
	encoder_logger(ENCODEREVENT_leftB);
}

void rightenc_event_A()
{
	encoder_logger(ENCODEREVENT_rightA);
}

void rightenc_event_B()
{
	encoder_logger(ENCODEREVENT_rightB);
}

void encoder_logger(const ENCODEREVENT e)
{

	static int currstate = 0;
	static int laststate = 0;

	laststate = currstate;

	if( (e == ENCODEREVENT_leftA) || (e == ENCODEREVENT_leftB) )
	{
		currstate = (digitalRead(left_encoder_pin_A) << 1) | digitalRead(left_encoder_pin_B);
		incr_or_dec(currstate, laststate, &left_ticks);
	}
	else
	{
		currstate = (digitalRead(right_encoder_pin_A) << 1) | digitalRead(right_encoder_pin_B);
		incr_or_dec(currstate, laststate, &right_ticks);
	}
}

//decode quadrature encoder input as a func of the current state and last state
//the state is defined as x in [0,3], with ch A being the top bit
void incr_or_dec(int currstate, int laststate, volatile int64_t* counter)
{
	/*
	http://www.mcmanis.com/chuck/robotics/projects/lab-x3/quadratrak.html
	http://en.wikipedia.org/wiki/Rotary_encoder

	 clockwise ->
	00 01 11 10 00
	   <- ccw

	need to verify cw/ccw dir on motor
	*/

	//int laststate = (A << 1) | B;
	//int currstate = (prevA << 1) | prevB;

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
				case 1: // 10 -> 01
					return;
				case 2: // 10 -> 10
					return;
				case 3: // 10 -> 11
					(*counter)--;
			}
		case 3: // 11
			switch(currstate)
			{
				case 0: // 11 -> 00
					return;
				case 1: // 11 -> 01
					(*counter)--;
				case 2: // 11 -> 10
					(*counter)++;
				case 3: // 11 -> 11
					return;
			}
	}
	
}

int incr_or_dec(int currstate, int laststate)
{
	/*
	http://www.mcmanis.com/chuck/robotics/projects/lab-x3/quadratrak.html
	http://en.wikipedia.org/wiki/Rotary_encoder

	 clockwise ->
	00 01 11 10 00
	   <- ccw
	*/

	//int laststate = (A << 1) | B;
	//int currstate = (prevA << 1) | prevB;

	switch(laststate)
	{
		case 0: // 00
			switch(currstate)
			{
				case 0: // 00 -> 00
					return 0;
				case 1: // 00 -> 01
					return 1;
				case 2: // 00 -> 10
					return -1;
				case 3: // 00 -> 11
					goto error;
			}
		case 1: // 01
			switch(currstate)
			{
				case 0: // 01 -> 00
					return -1;
				case 1: // 01 -> 01
					return 0;
				case 2: // 01 -> 10
					goto error;
				case 3: // 01 -> 11
					return 1;
			}
		case 2: // 10
			switch(currstate)
			{
				case 0: // 10 -> 00
					return 1;
				case 1: // 10 -> 01
					goto error;
				case 2: // 10 -> 10
					return 0;
				case 3: // 10 -> 11
					return -1;
			}
		case 3: // 11
			switch(currstate)
			{
				case 0: // 11 -> 00
					goto error;
				case 1: // 11 -> 01
					return -1;
				case 2: // 11 -> 10
					return 1;
				case 3: // 11 -> 11
					return 0;
			}
	}
	
	error:
	return 0xFF;

	#if 0
	switch(A)
	{
		case 0:
			switch(prevA)
			{
				case 0:
					switch(B)
					{
						case 0:
							switch(prevB)
							{
								case 0:// 00 -> 00
									goto error;
								case 1:// 01 -> 00
									goto error;
							}
						case 1:
							switch(prevB)
							{
								case 0:// 00 -> 01
									return -1
								case 1:// 01 -> 01
									return 0;
							}
					}
				case 1:
					switch(B)
					{
						case 0:
							switch(prevB)
							{
								case 0:// 10 -> 
									return -1;
								case 1:// 00 11
									goto error;
							}
						case 1:
							switch(prevB)
							{
								case 0:// 01 10
									goto error;
								case 1:// 01 11
									return -1;
							}
					}
			}
		case 1:
	}
	#endif
}

#if 0
void leftenc_event_c1()
{
	static bool lastvalue;
}

void leftenc_event_c2()
{
	static bool lastvalue;
}

void rightenc_event_c1()
{
	static bool lastvalue;
}

void rightenc_event_c2()
{
	static bool lastvalue;
}
#endif
