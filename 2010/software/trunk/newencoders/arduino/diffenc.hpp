#ifndef DIFFENC_HPP_
#define DIFFENC_HPP_

#include "WProgram.h"

#include <stdint.h>

enum ENCODEREVENT{ENCODEREVENT_leftA, ENCODEREVENT_leftB, ENCODEREVENT_rightA, ENCODEREVENT_rightB};

extern volatile int64_t left_ticks;
extern volatile int64_t right_ticks;

extern volatile int64_t left_d_ticks;
extern volatile int64_t right_d_ticks;

void leftenc_event_A();
void leftenc_event_B();
void rightenc_event_A();
void rightenc_event_B();

void encoder_logger(const ENCODEREVENT e);

void incr_or_dec(int currstate, int laststate, volatile int64_t* counter);
int incr_or_dec(int currstate, int laststate);

#endif

