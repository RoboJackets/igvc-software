#include "motors.h"

/*
 *	Refer to the header for more information about the purpose of each function
 *
 */

#define min(a, b) (((a) < (b)) ? (a) : (b))

/* Various values for formatting data to be communicated with the motor controller */
#define ERROR_MASK			0x3c
#define ERROR_OFFSET		2
#define CONTROL_MODE_MASK	0xc0
#define CONTROL_MODE_OFFSET	6
#define LEFT_DIR_MASK		0x01
#define LEFT_DIR_OFFSET		0
#define RIGHT_DIR_MASK		0x02
#define RIGHT_DIR_OFFSET	1
#define FORWARD				0
#define REVERSE				1

/* Posible errors recieved from the motor controller*/
enum {
	FRAME_ERROR = 1,
	DATA_OVERRUN_ERROR = 2,
	PARITY_ERROR = 4,
	TIMEOUT_ERROR = 8,
};

/* Defines way the robot is controlled */
typedef enum{
	AUTONOMOUS_MODE = 0,
	NAVIGATION_MODE = 1,
	JOYSTICK_MODE = 2,
	STOP_MODE = 3,
} CONTROLMODE_T;

//TODO:
//	make sure the two different types of motor commands (MOVE and VEL) don't mess each other up
//	maybe do multi-threading
//	add something to keep the message if the motor set fails
//	think about condensing the error reporting -maybe change warnings back to errors and add try catch blocks
//	add stop stuff - maybe use interface for other stop thing
//	fix PID
//	change names

Driver* motors::motors_Init(ConfigFile* cf, int section)
{
	return( (Driver*)(new motors(cf, section)) );
}

void motors::motors_Register(DriverTable* table)
{
	table->AddDriver("motors",  motors_Init);
	return;
}

motors::motors(ConfigFile* cf, int section) 
 : Driver(cf, section, TRUE, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_POSITION2D_CODE)
{
	/* Reset the file descriptor for the serial port */	fdMotor = -1;

	/* Read parameters from the configuration file */
	cpMotorPort = cf->ReadString(section, "port", DEFAULT_PORT);
	pGain = cf->ReadFloat(section, "PGain", DEFAULT_PGAIN);
	iGain = cf->ReadFloat(section, "IGain", DEFAULT_IGAIN);
	dGain = cf->ReadFloat(section, "DGain", DEFAULT_DGAIN);
	dVelocityScale = cf->ReadFloat(section, "Velocity Scale", DEFAULT_VELOCITY_SCALE);
	//dMaxSpeed
	//dMaxAccel

	/* Only recieve messages related to controlling the motors */
	//SetFilter();

	/* Sets PLAYER_POSITION2D_CMD_POS messages (commands to move to a position)
	 * to not be replaced by new messages of the same type.  All other command
	 * messages are replaced by newer ones (could be set otherwise if needed).
	 */
	InQueue->AddReplaceRule(device_addr, PLAYER_MSGTYPE_CMD, PLAYER_POSITION2D_CMD_POS, FALSE);

	return;
}

int motors::Setup()
{
	if(SetupSerial()){ return(-1); }
	StartThread();
	return(0);
}

int motors::Shutdown()
{
	StopThread();
	if(ShutdownSerial()){ return(-1); }
	return(0);
}

int motors::ProcessMessage(MessageQueue* resp_queue, player_msghdr* hdr, void* data) 
{
	if(hdr->type == PLAYER_MSGTYPE_REQ)
	{
		/* Turn the motors on and off - to be used in emergency situations when the robot
		 * needs to avoid running into something.
		 */
		if( Message::MatchMessage(hdr,
					PLAYER_MSGTYPE_REQ,
					PLAYER_POSITION2D_REQ_MOTOR_POWER,
					device_addr) )
		{
			return( set_motors(*(player_position2d_power_config_t *)data) );
		}
		/* Change the way velcoity commands are sent */
		else if( Message::MatchMessage(hdr,
					PLAYER_MSGTYPE_REQ,
					PLAYER_POSITION2D_REQ_VELOCITY_MODE,
					device_addr) )
		{
			return( change_velocity_mode(*(player_position2d_velocity_mode_config_t *)data) );
		}
		/* Choose between sending velocity commands or position commands */
		else if( Message::MatchMessage(hdr,
					PLAYER_MSGTYPE_REQ,
					PLAYER_POSITION2D_REQ_POSITION_MODE,
					device_addr) )
		{
			return( set_position_mode(*(player_position2d_position_mode_req_t *)data) );
		}
		/* Set velocity PID parameters */
		else if( Message::MatchMessage(hdr,
					PLAYER_MSGTYPE_REQ,
					PLAYER_POSITION2D_REQ_SPEED_PID,
					device_addr) )
		{
			return( set_pid((player_position2d_speed_pid_req_t *)data) );
		}

		/* Set maximum speed and accelleration */
		else if( Message::MatchMessage(hdr,
					PLAYER_MSGTYPE_REQ,
					PLAYER_POSITION2D_REQ_SPEED_PROF,
					device_addr))
		{
			return( set_max_move_param(*(player_position2d_speed_prof_req_t *)data) );
		}
		else
		{
			return(-1);
		}
	}
	else if(hdr->type == PLAYER_MSGTYPE_CMD)
	{
		/* This is a set of motor speeds that should be outputed */
		if( Message::MatchMessage(hdr,
					PLAYER_MSGTYPE_CMD,
					PLAYER_POSITION2D_CMD_VEL,
					device_addr) )
		{
			return( set_motors(*(player_position2d_cmd_vel_t *)data) );
		}
		/* This is a position to which the robot should move */
		else if( Message::MatchMessage(hdr,
					PLAYER_MSGTYPE_CMD,
					PLAYER_POSITION2D_CMD_POS,
					device_addr) )
		{
			return( queqe_position(*(player_position2d_cmd_pos_t *)data) );
		}
		else
		{
			return(-1);
		}
	}
	else
		return(-1);
}

void motors::Main()
{
	for(;;)
	{
		/* Test if the main thread should cancel */
		pthread_testcancel();

		/* Run ProcessMessage() for each message in the queqe */
		ProcessMessages();
	}
}

int motors::SetupSerial()
{
	/* Open serial port with the specified flags
	 * O_RDWR - for reading and writing
	 * O_NOCTTY - don't act as the controlling terminal for the port
	 * O_DSYNC | O_RSYNC - complete i/o operiations as definded by synchronized i/o data
	 *	integrity completion (i.e. waits for the data but not meta-data to be written
	 *	to the file before continuing execution) - this doesn't seem to work
	 * O_NDELAY - don't wait for the DCD line to go to the space voltage
	 */	if ((fdMotor = open(cpMotorPort, O_RDWR | O_NOCTTY | O_DSYNC | O_RSYNC | O_NDELAY)) < 0)
	{
		PLAYER_ERROR2("SetupSerial(): could not open port %s, %s\n", cpMotorPort, strerror(errno));
		return(-1);
	}

	/* Set the port back to blocking (waiting) behavior for calls to read() */
	fcntl(fdMotor, F_SETFL, 0);

	/* Get the current options for the port */
	struct termios options;
	if(tcgetattr(fdMotor, &options) < 0)
	{
		PLAYER_ERROR1("SetupSerial(): could not get the serial port options, %s\n", strerror(errno));
		ShutdownSerial();
		return(-1);
	}

	/* Configure the port options */	 
	options.c_cflag |= (CREAD | CLOCAL); /* Enable the receiver and set local mode */
	cfsetispeed(&options, B115200); /* Set 115200 baud rate */
	cfsetospeed(&options, B115200); 
	options.c_cflag &= ~PARENB; /* Set no parity */
	options.c_cflag &= ~CSTOPB; /* Set 1 stop bit */
	options.c_cflag &= ~CSIZE; /* Set 8 data bits */
	options.c_cflag |= CS8;
	options.c_cc[VMIN]  = 0; /* Set .2 sec timeout */
	options.c_cc[VTIME] = 2;
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); /* Set raw input and output */
	options.c_oflag &= ~OPOST;

	/* Flush input and output buffers and then set the new options for the port */
	if(tcsetattr(fdMotor, TCSAFLUSH, &options) < 0 )
	{
		PLAYER_ERROR1("SetupSerial(): could not set the serial port options, %s\n", strerror(errno));
		ShutdownSerial();
		return(-1);
	}
	return(0);
}

int motors::ShutdownSerial()
{
	if(close(fdMotor) < 0)
	{
		PLAYER_ERROR1("ShutdownSerial(): could not close the serial port, %s\n", strerror(errno));
		fdMotor = -1;
		return(-1);
	}
	fdMotor = -1;
	return(0);
}

int motors::set_pid(player_position2d_speed_pid_req_t *data)
{
	pGain = data->kp;
	iGain = data->ki;
	dGain = data->kd;
	return(0);
}

int motors::move(void)
{
#ifdef _WOOT__MOVE_IS_GOOD_
	POSTION_T dIState, dDState, output; // intialize these

	while() // TODO: when should this end?
	{
		if(bStop)
		{
			return(-1);
		}

		/* Get the current posistion and error */
		POSTION_T position = sRobotPosition;
		POSTION_T error = position - output;

		/* calculate the proportional term and assign it to the output */
		output = dPGain * error;

		/* calculate the integral state with appropriate limiting and add it to the output */
		dIState += error;
		if(dIState > dIMax)
			dIState = dIMax;
		else if(dIState < dIMin)
			dIState = dIMin;
		output += dIGain * dIState;

		/* calculate the differential term and subtract it from the output */
		output -= dDGain * (postion - dDState);
		dDState = position;

		/* Calculate the motor velocities based on the output */
		MOTOR_STATES_T stNewMotorStates;
		r = sqrt()
		stNewMotorStates.iLeftVelocity = output.x * output.theta; //fix**********
		stNewMotorStates.iRightVelocity = output.x * output.theta;

		/* Set the motors to the calculated velocities */
		setmotors(/*?*/);
	}
#endif
	return(0);
}

/* NOTE:
 *	When you raise the DTR line, if the motor controller is in AUTONOMOUS_MODE,
 *	it reads in any data in its uart buffer (3 bytes in size). Then, in all modes,
 *	it sends back its current motor states, control mode, and any errors it had
 *	with the transmission.
 */

int motors::set_motors(player_position2d_cmd_vel_t stVelocity)
{
	//printf("LeftVel command: %f\n", stVelocity.vel.px);
	//printf("RightVel command: %f\n\n", stVelocity.vel.pa);

	/* Scale the motor velocities, limit them to vaild values, and store them in the
	 * correct form to transmit to the motor controller.
	 */
	int iScaledLeftVel = (int)(dVelocityScale * stVelocity.vel.px);
	int iScaledRightVel = (int)(dVelocityScale * stVelocity.vel.pa);
	unsigned char data[3] ={
		(unsigned char)( (((iScaledLeftVel < 0) ? REVERSE : FORWARD) << LEFT_DIR_OFFSET) | (((iScaledRightVel < 0) ? REVERSE : FORWARD) << RIGHT_DIR_OFFSET) ),
		(unsigned char)( min(abs(iScaledLeftVel), 255) ),
		(unsigned char)( min(abs(iScaledRightVel), 255) )  };

	/* Send the motor states to the motor controller */
	if( write(fdMotor, data, 3) != 3 )
	{
		PLAYER_WARN1("set_motors(): could not write to serial port, %s\n", strerror(errno));
		return(-1);
	}

	/* Make sure the data has finished being written - this can be removed if the option can be made work in open() */
	tcdrain(fdMotor);

	/* Get the current motor states from the motor controller */
	return( get_motor_states() );
}

int motors::set_motors(player_position2d_power_config_t stPower)
{
	#if 0 // To be used once position commands are accepted
	if(stPower.state == false)
	{
		set_motors( (player_position2d_cmd_vel_t){{0, 0, 0}, FALSE} );
		stopped = true;
	}
	else
	{
		stopped = false;
	}
	#endif
	return(0);
}

int motors::get_motor_states(void)
{
	/* Flush any old data in the read buffer */
	tcflush(fdMotor, TCIFLUSH);

	/* Set the DTR line high to intterupt the motor controller */
	int status;
	if(ioctl(fdMotor, TIOCMGET, &status))
	{
		PLAYER_WARN1("get_motor_states(): could not get serial port status, %s\n", strerror(errno));
		return(-1);
	}
	status |= TIOCM_DTR;
	if(ioctl(fdMotor, TIOCMSET, &status))
	{
		PLAYER_WARN1("get_motor_states(): could not set RTS high, %s\n", strerror(errno));
		return(-1);
	}
	
	/* 
	 * If the DTR line can be raised and lowered in less than a full clock cycle
	 * of the controller, there should be a delay here to make sure the motor
	 * controller interrupts.
	 */

	/* Set the DTR line back to low
	 *	If there are problems, you might need to get the motor status again with
	 *	ioctl(fdMotor, TIOCMGET, &status) before doing this.
	 */
	status &= ~TIOCM_DTR;
	if(ioctl(fdMotor, TIOCMSET, &status))
	{
		PLAYER_WARN1("get_motor_states(): could not set RTS low, %s\n", strerror(errno));
		return(-1);
	}

	/* Read each byte sent from the controller */
	unsigned char u8Buff[3];
	int bytesRead;
	if((bytesRead = read(fdMotor, u8Buff, 3)) != 3)
	{
		PLAYER_WARN1("get_motor_states(): the motor controller did not respond or only partial responded, %s\n", strerror(errno));
		return(-1);
	}

	//Test
	/*printf("%d bytes read\n", bytesRead);
	for(int i = 0; i < bytesRead; i++)
	{
		printf("%d ", u8Buff[i]);
	}
	printf("\n\n");*/

	/* Report any errors the motor controller had with the transmission */
	int iError = (u8Buff[0] & ERROR_MASK) >> ERROR_OFFSET;
	if( iError & (FRAME_ERROR|DATA_OVERRUN_ERROR|PARITY_ERROR|TIMEOUT_ERROR) )
	{
		PLAYER_WARN1("get_motor_states(): the motor controller could not process comand, Error %d\n", iError);
		return(-1);
	}
	

	/* Store the current control mode */
	controlMode = ( u8Buff[0] & CONTROL_MODE_MASK ) >> CONTROL_MODE_OFFSET;

	/* Store the current motor velocities */
	stMotorStates.iLeftVelocity = ((((u8Buff[0] & LEFT_DIR_MASK) >> LEFT_DIR_OFFSET)==FORWARD) ? 1 : -1) * (int)u8Buff[1];
	stMotorStates.iRightVelocity = ((((u8Buff[0] & RIGHT_DIR_MASK) >> RIGHT_DIR_OFFSET)==FORWARD)? 1 : -1) * (int)u8Buff[2];

	return(0);
}

int change_velocity_mode(player_position2d_velocity_mode_config_t data)
{
	return(0);
}

int motors::set_position_mode(player_position2d_position_mode_req_t data)
{
	return(0);
}

int motors::set_max_move_param(player_position2d_speed_prof_req_t data)
{
	return(0);
}


int motors::queqe_position(player_position2d_cmd_pos_t stPosition)
{
	return(0);
}


/* Extra stuff for building a shared object.
 * need the extern to avoid C++ name-mangling
 */
extern "C" {
	int player_driver_init(DriverTable* table)
	{
		puts("motors driver initializing");
		motors::motors_Register(table);
		puts("motors driver done");
		return(0);
	}
}
