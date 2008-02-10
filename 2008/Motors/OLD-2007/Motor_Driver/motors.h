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

#include "motordriver.h"

/* Values that will be used if they are not specified in the configuration file */
#define DEFAULT_PORT		"/dev/ttyS0"
#define DEFAULT_PGAIN		1
#define DEFAULT_IGAIN		.001
#define DEFAULT_DGAIN		1
#define DEFAULT_VELOCITY_SCALE	1

/* The driver class */
class Motors_Player: public Driver
{
	public:
		/* Initialization function */
		static Driver* Motors_Player_Init(ConfigFile* cf, int section);

		/* Driver registration function */
		static void Motors_Player_Register(DriverTable* table);

		/* Constructor */
		Motors_Player(ConfigFile* cf, int section);

		/* Called when the driver is started and closed respectively */
		virtual int Setup();
		virtual int Shutdown();

		/* Called on each incoming message - sends each message to the approriate function */
		virtual int ProcessMessage(MessageQueue* resp_queue, player_msghdr* hdr, void* data);

	protected:
		/* This is run as a thread */
		virtual void Main();
		
	private:
		int SetMotorPID(player_position2d_speed_pid_req_t* data);
		/* Set the motor velocities to the specified value */
		int SetMotorSpeeds(player_position2d_cmd_vel_t* stVelocity);
		
	private:
		// The actual motor driver
		Motors* driver;
		
		/* Scales external velocity representations to interal ones */
		double dVelocityScale;
};

#endif /* _MOTORS_H_ */

