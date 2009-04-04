#include "motors_old.h"

#define MINREQSPEED 45

Motors_Old::Motors_Old()
{
	//ctor
	fdMotor = -1;
	dVelocityScale = 1;
	_max_speed_=100;
	//SetupSerial();
}

Motors_Old::~Motors_Old()
{
	//dtor
	ShutdownSerial();
}



int Motors_Old::SetupSerial()
{
	/* Open serial port with the specified flags
	 * O_RDWR - for reading and writing
	 * O_NOCTTY - don't act as the controlling terminal for the port
	 * O_DSYNC | O_RSYNC - complete i/o operiations as definded by synchronized i/o data
	 *	integrity completion (i.e. waits for the data but not meta-data to be written
	 *	to the file before continuing execution) - this doesn't seem to work
	 * O_NDELAY - don't wait for the DCD line to go to the space voltage
	 */
	if ((fdMotor = open(DEFAULT_PORT, O_RDWR | O_NOCTTY | O_DSYNC | O_RSYNC | O_NDELAY)) < 0)
	{
		printf("SetupSerial(): could not open port %s, %s\n", DEFAULT_PORT, strerror(errno));
		return(-1);
	}

	/* Set the port back to blocking (waiting) behavior for calls to read() */
	fcntl(fdMotor, F_SETFL, 0);

	/* Get the current options for the port */
	struct termios options;
	if (tcgetattr(fdMotor, &options) < 0)
	{
		printf("SetupSerial(): could not get the serial port options, %s\n", strerror(errno));
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
	if (tcsetattr(fdMotor, TCSAFLUSH, &options) < 0 )
	{
		printf("SetupSerial(): could not set the serial port options, %s\n", strerror(errno));
		ShutdownSerial();
		return(-1);
	}
	return(0);
}

int Motors_Old::ShutdownSerial()
{
	if (close(fdMotor) < 0)
	{
		printf("ShutdownSerial(): could not close the serial port, %s\n", strerror(errno));
		fdMotor = -1;
		return(-1);
	}
	fdMotor = -1;
	return(0);
}

int Motors_Old::Shutdown()
{
	return(ShutdownSerial());
}

/* NOTE:
 *	When you raise the DTR line, if the motor controller is in AUTONOMOUS_MODE,
 *	it reads in any data in its uart buffer (4 bytes at most). Then, in all modes,
 *	it sends back its current motor states, control mode, and any errors it had
 *	with the transmission.
 */
int Motors_Old::set_motors(int iLeftVelocity, int iRightVelocity)
{
	/* Scale the motor velocities, limit them to vaild values, and store them in the
	 * correct form to transmit to the motor controller.
	 * 	Alternate Title: fun with the ? operator
	 */
	int iScaledLeftVel = (int)(dVelocityScale * iLeftVelocity);
	int iScaledRightVel = (int)(dVelocityScale * iRightVelocity);
	unsigned char data[3] =
	{
		(unsigned char)( (((iScaledLeftVel < 0) ? REVERSE : FORWARD) << LEFT_DIR_OFFSET) | (((iScaledRightVel < 0) ? REVERSE : FORWARD) << RIGHT_DIR_OFFSET) ),
		(unsigned char)( min(abs(iScaledLeftVel), 255) ),
		(unsigned char)( min(abs(iScaledRightVel), 255) )
	};

	/* Send the motor states to the motor controller */
	if ( write(fdMotor, data, 3) != 3 )
	{
		printf("set_motors(): could not write to serial port, %s\n", strerror(errno));
		return(-1);
	}

	/* Make sure the data has finished being written - this can be removed if the option can be made work in open() */
	tcdrain(fdMotor);

	/* Get the current motor states from the motor controller */
	return( get_motor_states() );
}

int Motors_Old::get_motor_states(void)
{
	/* Flush any old data in the read buffer */
	tcflush(fdMotor, TCIFLUSH);

	/* Set the DTR line high to intterupt the motor controller */
	int status;
	if (ioctl(fdMotor, TIOCMGET, &status))
	{
		printf("get_motor_states(): could not get serial port status, %s\n", strerror(errno));
		return(-1);
	}
	status |= TIOCM_DTR;
	if (ioctl(fdMotor, TIOCMSET, &status))
	{
		printf("get_motor_states(): could not set RTS high, %s\n", strerror(errno));
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
	if (ioctl(fdMotor, TIOCMSET, &status))
	{
		printf("get_motor_states(): could not set RTS low, %s\n", strerror(errno));
		return(-1);
	}

	/* Read each byte sent from the controller */
	unsigned char u8Buff[3];
	int bytesRead;
	if ((bytesRead = read(fdMotor, u8Buff, 3)) != 3)
	{
		printf("get_motor_states(): the motor controller did not respond or only partial responded, %s\n", strerror(errno));
		//return(-1);
	}

	//Test
	printf("%d bytes read\n", bytesRead);
	for (int i = 0; i < bytesRead; i++)
	{
		printf("%d ", u8Buff[i]);
	}
	printf("\n");

	/* Report any errors the motor controller had with the transmission */
	int iError = (u8Buff[0] & ERROR_MASK) >> ERROR_OFFSET;
	if ( iError & (FRAME_ERROR|DATA_OVERRUN_ERROR|PARITY_ERROR|TIMEOUT_ERROR) )
	{
		printf("get_motor_states(): the motor controller could not process comand, Error %d\n", iError);
		return(-1);
	}

	/* Store the current control mode */
	controlMode = ( u8Buff[0] & CONTROL_MODE_MASK ) >> CONTROL_MODE_OFFSET;

	/* Store the current motor velocities */
	_iLeftVelocity = ((((u8Buff[0] & LEFT_DIR_MASK) >> LEFT_DIR_OFFSET)==FORWARD) ? 1 : -1) * (int)u8Buff[1];
	_iRightVelocity = ((((u8Buff[0] & RIGHT_DIR_MASK) >> RIGHT_DIR_OFFSET)==FORWARD)? 1 : -1) * (int)u8Buff[2];

	return(0);
}

/* Set motors based on given velocity:
 * Fwd range [0,255]
 * Rot range [-128,127]
 */
int Motors_Old::set_heading(int iFwdVelocity, int iRotation)
{
	iRotation *= 1.10; // XXX: hack

	int left  = iFwdVelocity + iRotation ;
	int right = iFwdVelocity - iRotation ;

	if (1)
	{
		// scale speed
		left  = ( left  * _max_speed_ / 255 ) ;
		right = ( right * _max_speed_ / 255 ) ;
	}
	else
	{
		// cap speed
		if (left  > _max_speed_) left  = _max_speed_ ;
		if (right > _max_speed_) right = _max_speed_ ;
	}

	// motors don't respond until certain output is reached
	if (right != 0) right += MINREQSPEED ;
	if (left  != 0) left  += MINREQSPEED ;

	printf("Motors: L=%d R=%d \n",left,right);

	return this->set_motors( left , right );
}


void Motors_Old::set_max_speed(int maxspeed)
{
	/* 0-255 */
	_max_speed_ = maxspeed;
}


