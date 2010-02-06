#ifndef MOTORS_OLD_H
#define MOTORS_OLD_H

#include "OSMC_driver.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <iostream>

/* maximum output value out of 255 */
//#define MAX_OUTPUT          90

#define DEFAULT_PORT		"/dev/ttyUSB0"

/* Various values for formatting data to be communicated with the motor controller */
#define ERROR_MASK		    0x3c
#define ERROR_OFFSET		2
#define CONTROL_MODE_MASK	0xc0
#define CONTROL_MODE_OFFSET	6
#define LEFT_DIR_MASK		0x01
#define LEFT_DIR_OFFSET		0
#define RIGHT_DIR_MASK		0x02
#define RIGHT_DIR_OFFSET	1
#define FORWARD			    1
#define REVERSE			    0
#define min(a, b) (((a) < (b)) ? (a) : (b))

/* Posible errors recieved from the motor controller*/
enum
{
	FRAME_ERROR = 1,
	DATA_OVERRUN_ERROR = 2,
	PARITY_ERROR = 4,
	TIMEOUT_ERROR = 8,
};


/* Motor drive control */
//typedef struct _motor_states_
//{
//    int iLeftVelocity, iRightVelocity;
//} MOTOR_STATES_T;


class Motors_Old
{
public:

	/* Constructor */
	Motors_Old();
	/* Destructor */
	virtual ~Motors_Old();
	/* Called when the driver is started and closed respectively */
	int Shutdown();
	/* file descriptor of the serial port */
	int fdMotor;
	/**/
	int SetupSerial();
	/**/
	int ShutdownSerial();
	/* Current motor velocities */
	//MOTOR_STATES_T stMotorStates;
	int _iLeftVelocity;
	int _iRightVelocity;
	/**/
	float dVelocityScale;
	/**/
	int controlMode;
	/**/
	int set_motors(int iLeftVelocity, int iRightVelocity);
	int set_heading(int iFwdVelocity, int iRotation);
	/**/
	int get_motor_states(void);
	/**/
	int _max_speed_;
	void set_max_speed(int maxspeed);

};

#endif // MOTORS_OLD_H

