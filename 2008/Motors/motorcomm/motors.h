#ifndef MOTORS_H
#define MOTORS_H

#include <stdlib.h>
#include <termios.h>  // for speed_t
#include <stdbool.h>  // for bool

//The structure motor_reply_t defines and receives data that is read from the arduino-based motor control board.
typedef struct {
	uint8_t HWESTOP; //0 if Hardware Estop is untriggered, 1 if it is
	uint8_t AUTOMAN; //0 if the robot is operating in autonomous mode, 1 if manual control is active
	uint8_t PATHNAV; //0 if the robot is in autonomous path following mode, 1 if the robot is in autonomous waypoint navigation mode
	uint8_t CURRENTLEFT1; //Upper byte of the left motor current sensor's 10 bit reading
	uint8_t CURRENTLEFT2; //Lower byte of the left motor current sensor's 10 bit reading
	uint8_t CURRENTRIGHT1; //Upper byte of the right motor current sensor's 10 bit reading
	uint8_t CURRENTRIGHT2; //Lower byte of the right motor current sensor's 10 bit reading
	uint8_t LOGICBATT1; //Upper byte of the logic battery system's voltage 10 bit reading
	uint8_t LOGICBATT2; //Lower byte of the logic battery system's voltage 10 bit reading
	uint8_t MOTORBATT1; //Upper byte of the motor battery system's voltage 10 bit reading
	uint8_t MOTORBATT2; //Lower byte of the motor battery system's voltage 10 bit reading
	uint8_t LEFTMOTORSPEED1; //Upper byte of the left motor battery speed 10 bit reading
	uint8_t LEFTMOTORSPEED2; //Lower byte of the left motor battery speed 10 bit reading
	uint8_t RIGHTMOTORSPEED1; //Upper byte of the right motor battery speed 10 bit reading
	uint8_t RIGHTMOTORSPEED2; //Lower byte of the right motor battery speed 10 bit reading
} motor_reply_t;

typedef enum {
	/**
	 * leftMotorSpeed and rightMotorSpeed can have values between 0 and 255 inclusive.
	 * A value of 128 stops the motors. "Positive" values (>128) spin the motors forward.
	 * "Negative" values (<128) spin the motors in reverse.
	 */
	MV_LEFT_MOTOR_SPEED		= 0,
	MV_RIGHT_MOTOR_SPEED	= 1,
	/**
	 * Setting softEStop to anything other than 0 makes the motors stop moving and
	 * sets their speed to 128 (stopped).
	 */
	MV_SOFT_ESTOP			= 2,
} motor_var_t;

bool motors_open(void);
void motors_close(void);
bool motors_getStatus(motor_reply_t *status);
bool motors_setVar(motor_var_t var, char value);

#endif /* MOTORS_H */
