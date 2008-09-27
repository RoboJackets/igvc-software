#ifndef ENCODER_DEFINES_H_
#define ENCODER_DEFINES_H_

/* MOTOR ENCODER DEFINITIONS */
#define COUNTER_SCALER			(64)
#define COUNTER_RATE			(F_CPU/COUNTER_SCALER)
#define TOTAL_ENCODER_TICKS 		((double)512)
#define RAD_PER_ENCODER_TICK		((double)TWO_PI/TOTAL_ENCODER_TICKS)
#define LEFT_MOTOR_ENCODER_DIRECTION 	1
#define RIGHT_MOTOR_ENCODER_DIRECTION 	0

/* ROBOT PHYSICAL DEFINITIONS */  /*TODO: determine these values */
#define MOTOR_RATIO		((double)20)
#define WHEEL_RADIUS		((double)5 / (double)MOTOR_RATIO)
#define WHEEL_BASE		((double)28)

// Enums for data settings
enum var_t { PUSHPULL, RET_T, INTEROG_DL};//interogdl - miliseconds
enum send_t {PUSH = 0, PULL};
enum ret_t { SEND_DTICK, SEND_CURRENT };

#endif //ENCODER_DEFINES_H_
