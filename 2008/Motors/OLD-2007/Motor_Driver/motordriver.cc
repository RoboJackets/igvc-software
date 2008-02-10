#include "motordriver.h"

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

//TODO:
//	make sure the two different types of motor commands (MOVE and VEL) don't mess each other up
//	maybe do multi-threading
//	add something to keep the message if the motor set fails
//	think about condensing the error reporting -maybe change warnings back to errors and add try catch blocks
//	add stop stuff - maybe use interface for other stop thing
//	fix PID
//	change names

Motors::Motors(
	const char* cpMotorPort,
	double pGain, double iGain, double dGain)
{
	/* Reset the file descriptor for the serial port */	this->fdMotor = -1;

	/* Read parameters from the configuration file */
	this->cpMotorPort = cpMotorPort;
	this->pGain = pGain;
	this->iGain = iGain;
	this->dGain = dGain;
}

bool Motors::Connect()
{
	if(SetupSerial()){ return false; }
	return true;
}

bool Motors::Disconnect()
{
	if(ShutdownSerial()){ return false; }
	return true;
}

void Motors::SetPID(double pGain, double iGain, double dGain)
{
	this->pGain = pGain;
	this->iGain = iGain;
	this->dGain = dGain;
}

int Motors::SetupSerial()
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

int Motors::ShutdownSerial()
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

/* NOTE:
 *	When you raise the DTR line, if the motor controller is in AUTONOMOUS_MODE,
 *	it reads in any data in its uart buffer (3 bytes in size). Then, in all modes,
 *	it sends back its current motor states, control mode, and any errors it had
 *	with the transmission.
 */

bool Motors::SetSpeeds(int leftVel, int rightVel)
{
	int iScaledLeftVel = leftVel;
	int iScaledRightVel = rightVel;
	unsigned char data[3] ={
		(unsigned char)( (((iScaledLeftVel < 0) ? REVERSE : FORWARD) << LEFT_DIR_OFFSET) | (((iScaledRightVel < 0) ? REVERSE : FORWARD) << RIGHT_DIR_OFFSET) ),
		(unsigned char)( min(abs(iScaledLeftVel), 255) ),
		(unsigned char)( min(abs(iScaledRightVel), 255) )  };

	/* Send the motor states to the motor controller */
	if( write(fdMotor, data, 3) != 3 )
	{
		PLAYER_WARN1("set_motors(): could not write to serial port, %s\n", strerror(errno));
		return false;
	}

	/* Make sure the data has finished being written - this can be removed if the option can be made work in open() */
	tcdrain(fdMotor);

	/* Get the current motor states from the motor controller */
	return GetMotorStates();
}

bool Motors::GetMotorStates(void)
{
	/* Flush any old data in the read buffer */
	tcflush(fdMotor, TCIFLUSH);

	/* Set the DTR line high to interrupt the motor controller */
	int status;
	if(ioctl(fdMotor, TIOCMGET, &status))
	{
		PLAYER_WARN1("get_motor_states(): could not get serial port status, %s\n", strerror(errno));
		return false;
	}
	status |= TIOCM_DTR;
	if(ioctl(fdMotor, TIOCMSET, &status))
	{
		PLAYER_WARN1("get_motor_states(): could not set RTS high, %s\n", strerror(errno));
		return false;
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
		return false;
	}

	/* Read each byte sent from the controller */
	unsigned char u8Buff[3];
	int bytesRead;
	if((bytesRead = read(fdMotor, u8Buff, 3)) != 3)
	{
		PLAYER_WARN1("get_motor_states(): the motor controller did not respond or only partial responded, %s\n", strerror(errno));
		return false;
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
		return false;
	}
	

	/* Store the current control mode and motor velocities */
	motorStates.controlMode = ( u8Buff[0] & CONTROL_MODE_MASK ) >> CONTROL_MODE_OFFSET;
	motorStates.leftVel = ((((u8Buff[0] & LEFT_DIR_MASK) >> LEFT_DIR_OFFSET)==FORWARD) ? 1 : -1) * (int)u8Buff[1];
	motorStates.rightVel = ((((u8Buff[0] & RIGHT_DIR_MASK) >> RIGHT_DIR_OFFSET)==FORWARD)? 1 : -1) * (int)u8Buff[2];

	return true;
}

/* Extra stuff for building a shared object.
 * need the extern to avoid C++ name-mangling
 */
extern "C" {
	int player_driver_init(DriverTable* table)
	{
		puts("motors driver initializing");
		Motors_Player::Motors_Player_Register(table);
		puts("motors driver done");
		return(0);
	}
}
