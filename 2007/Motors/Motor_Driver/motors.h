#ifndef _MOTORS_H_
#define _MOTORS_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>

#include <libplayercore/playercore.h>

/* Values that will be used if they are not specified in the configuration file */
#define DEFAULT_PORT		"/dev/ttyS0"
#define DEFAULT_PGAIN		1
#define DEFAULT_IGAIN		.001
#define DEFAULT_DGAIN		1
#define DEFAULT_VELOCITY_SCALE	1

/* Used to internally represent the motor velocities and possibly other motor information */
typedef struct _motor_states_
{
	int iLeftVelocity, iRightVelocity;
} MOTOR_STATES_T;

/* The driver class */
class motors: public Driver
{
	public:
		/* Initialization function */
		static Driver* motors_Init(ConfigFile* cf, int section);

		/* Driver registration function */
		static void motors_Register(DriverTable* table);

		/* Constructor */
		motors(ConfigFile* cf, int section);

		/* Called when the driver is started and closed respectively */
		virtual int Setup();
		virtual int Shutdown();

		/* Called on each incoming message - sends each message to the approriate function */
		virtual int ProcessMessage(MessageQueue* resp_queue, player_msghdr* hdr, void* data);

	protected:
		/* This is run as a thread */
		virtual void Main();

		/* String name of serial port to use */
		const char* cpMotorPort;

		/* File descriptor of the serial port */
		int fdMotor;

		/* Called in Setup() and Shutdown() to open and close the serial port */
		int SetupSerial();
		int ShutdownSerial();

		/* Current motor velocities */
		MOTOR_STATES_T stMotorStates;

		/* PID controller parameters */
		double pGain, iGain, dGain;

		/* Set the PID controller parameters */
		int set_pid(player_position2d_speed_pid_req_t *data);

		/* Scales external velocity representations to interal ones */
		double dVelocityScale;

		/* Current postion, recieved from the ___ driver */
		player_pose_t stRobotPosition;

		/* Current mode the motor controller is in - either JOYSTICK_MODE or AUTONOMOUS_MODE */
		int controlMode;

		/* Use PID control to set the motor velocities to reach the specified position */
		int move(void);

		/* Set the motor velocities to the specified value */
		int set_motors(player_position2d_cmd_vel_t stVelocity);

		/* Same a previous, but uses PID to do motor breaking until the robot is stopped */
		int set_motors(player_position2d_power_config_t stPower);

		/* Retrieves the current motor states (velocities and other control data)
		 * - is always called when the motor velocities are set
		 */
		int get_motor_states(void);

		//
		//not implemented yet
		//

		/* change between the different velocity modes */
		int change_velocity_mode(player_position2d_velocity_mode_config_t data);

		/* Change between position mode and velocity mode */
		int set_position_mode(player_position2d_position_mode_req_t data);

		/**/
		int set_max_move_param(player_position2d_speed_prof_req_t data);

		/**/
		int queqe_position(player_position2d_cmd_pos_t stPosition);
};

#endif /* _MOTORS_H_ */

