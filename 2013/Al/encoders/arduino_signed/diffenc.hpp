#ifndef DIFFENC_HPP_
#define DIFFENC_HPP_

#include "WProgram.h"

#include <stdint.h>

enum ENCODEREVENT{ENCODEREVENT_leftA, ENCODEREVENT_leftB, ENCODEREVENT_rightA, ENCODEREVENT_rightB};

//extern volatile int64_t left_coder_ticks;
//extern volatile int64_t right_coder_ticks;
extern volatile int64_t coder_ticks;

void leftenc_event();
void rightenc_event();
void encoder_logger(int trigger);
void incr_or_dec(const int currstate, const int laststate, volatile int64_t* counter);

//decode quadrature encoder input as a func of the current state and last state
//the state is defined as x in {0x00, 0x4, 0x8, 0xC}, with ch A being the top bit
// 0: 0x00
// A: 0x04
// B: 0x08
// A&B: 0x0C
void encoder_logger();
void incr_or_dec_fast(const int currstate, const int laststate);

#endif

