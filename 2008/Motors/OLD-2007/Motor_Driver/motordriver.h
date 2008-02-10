#ifndef _MOTOR_DRIVER_H_
#define _MOTOR_DRIVER_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>

/* Defines way the robot is controlled */
typedef enum {
	AUTONOMOUS_MODE = 0,
	NAVIGATION_MODE = 1,
	JOYSTICK_MODE = 2,
	STOP_MODE = 3,
} MotorControlMode;

/* Used to internally represent the motor velocities and possibly other motor information */
typedef struct
{
	// Current mode the motor controller is in
	MotorControlMode controlMode;
	int leftVel, rightVel;
} MotorStates;

/* The driver class */
class Motors
{
	public:
		/* Constructor */
		Motors(
			const char* cpMotorPort,
			double pGain, double iGain, double dGain);
		Motors(const char* cpMotorPort) : Motors(cpMotorPort, 0, 0, 0);

		/** Connects to the motors. Returns true if successful. */
		bool Connect();
		/** Disconnects from the motors. Returns true if successful. */
		bool Disconnect();
		
		void SetPID(double pGain, double iGain, double dGain);

		/** Changes the speeds of the motors. Returns true if successful. */
		bool SetSpeeds(int leftVel, int rightVel);
	
	private:
		int SetupSerial();
		int ShutdownSerial();

		/**
		 * Gets the current motor states (velocities and other control data).
		 * Returns true if successful.
		 */
		bool GetMotorStates(void);
		
	private:
		// String name of serial port to use
		const char* cpMotorPort;
		// PID controller parameters
		double pGain, iGain, dGain;
		
		// File descriptor of the serial port
		int fdMotor;

		// Current motor velocities
		MotorStates motorStates;
};

#endif /* _MOTOR_DRIVER_H_ */

