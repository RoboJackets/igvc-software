
#include "candii_serial.h"
//#include "candii_motorcomm.h"
#include <stdio.h>
#include <SDL.h>

#define WAIT_TIME 20 //Used for setting VTIME, each tick is 0.1 s
#define BAUD B9600  //Serial baudrate
#define DEFAULT_SERIAL "/dev/ttyUSB0"
#define TANK_SAFETY_BUTTON 1
#define JOY_SAFETY_BUTTON 2
#define LOWER_JOY_BOUND -32768


int main(int argc, char *argv[]) {
	int i;
	int done;
	int buttons = 0;
	int dPadX = 0;
	int dPadY = 0;
	int leftAnalogX = 0;
	int leftAnalogY = 0;
	int rightAnalogX = 0;
	int rightAnalogY = 0;
	char joyStickUpdated = 0;
	int leftMotorSpeed=128;
	int rightMotorSpeed=128;
	SDL_Event event;
	SDL_Joystick *joystick;
	int fd = 0;
	fd = serialport_init(DEFAULT_SERIAL, BAUD);

	//setup starts:
	/* Initialize SDL */
	if ( SDL_Init(SDL_INIT_VIDEO|SDL_INIT_JOYSTICK) < 0 ) {
		fprintf(stderr, "Couldn't initialize SDL: %s\n",SDL_GetError());
		exit(1);
	}
	joystick = SDL_JoystickOpen(0);
	if (SDL_JoystickOpened(0)) {
		printf("Joystick Opened!\n");
	}
	else {
		printf("Failed to open joystick!\n");
		return 1;
	}
	done=0;
	for (i=1;i<100;i++) {
		SDL_PollEvent(&event);
	}
	printf("READY!\n");
	
	while ( ! done ) {
		if (joyStickUpdated) {
			printf("\n");
			if (buttons & TANK_SAFETY_BUTTON) {
				//this should scale it:
				printf("Tank drive engaged \n");
				leftMotorSpeed=(leftAnalogY-LOWER_JOY_BOUND)/256;
				rightMotorSpeed=(rightAnalogY-LOWER_JOY_BOUND)/256;
				printf("rightMotorSpeed= %d \n", rightMotorSpeed);
				printf("leftMotorSpeed= %d \n", leftMotorSpeed);
			}
			else {
				if (buttons & JOY_SAFETY_BUTTON) {
					int speed;
					int turn;
					speed=(rightAnalogY-LOWER_JOY_BOUND)/512;
					turn=rightAnalogX/512;
					leftMotorSpeed=speed+turn;
					rightMotorSpeed=speed-turn;
					printf("rightMotorSpeed= %d \n", rightMotorSpeed);
					printf("leftMotorSpeed= %d \n", leftMotorSpeed);
				}
				else {
					leftMotorSpeed=128;
					rightMotorSpeed=128;
					printf("Safety engaged.\n");
				}
			}
			char buf[3];
			buf[0]='w';
			buf[1]=0;
			buf[2]=leftMotorSpeed;
			if (writeFully(fd, buf, sizeof(buf)))
			{
				printf("serialport_write: Write error %d: %s\n", (int) errno, (char*) strerror(errno));
			}
			buf[0]='w';
			buf[1]=1;
			buf[2]=rightMotorSpeed;
			if (writeFully(fd, buf, sizeof(buf)))
			{
				printf("serialport_write: Write error %d: %s\n", (int) errno, (char*) strerror(errno));
			}
		}
		while ( SDL_PollEvent(&event) ) {
			switch (event.type) {
			    case SDL_JOYAXISMOTION:
				switch (event.jaxis.axis) {
					case 4:
						dPadX=event.jaxis.value;
					break;
					case 5:
						dPadY=event.jaxis.value;
					break;
					case 0:
						leftAnalogX=event.jaxis.value;
						joyStickUpdated=1;
					break;
					case 1:
						leftAnalogY=event.jaxis.value;
						joyStickUpdated=1;
					break;
					case 2:
						rightAnalogX=event.jaxis.value;
						joyStickUpdated=1;
					break;
					case 3:
						rightAnalogY=event.jaxis.value;
						joyStickUpdated=1;
					break;
					default:
					break;
				}
				break;
			    case SDL_JOYBUTTONDOWN:
				       buttons = buttons | 1<<event.jbutton.button;
						joyStickUpdated=1;
				break;
			    case SDL_JOYBUTTONUP:
				       buttons = buttons & ~1<<event.jbutton.button;
						joyStickUpdated=1;
				break;
			    default:
				break;
			}
		}
		return 0;
	}
}
	
int serialport_init(const char* serialport, int baud) {
	struct termios toptions;
	int fd;
	
	//fprintf(stderr,"init_serialport: opening port %s @ %d bps\n",
	//		serialport,baud);

	fd = open(serialport, (O_RDWR | O_NOCTTY | O_NDELAY) &~ O_NONBLOCK);
	if (fd == -1)  {
		perror("serialport_init: Unable to open port ");
		return -1;
	}
	
	if (tcgetattr(fd, &toptions) < 0) {
		perror("serialport_init: Couldn't get term attributes");
		return -1;
	}
	speed_t brate = baud; // let you override switch below if needed

	
    cfsetispeed(&toptions, baud);
	cfsetospeed(&toptions, baud);

	// 8N1
	toptions.c_cflag &= ~PARENB;
	toptions.c_cflag &= ~CSTOPB;
	toptions.c_cflag &= ~CSIZE;
	toptions.c_cflag |= CS8;
	
	// no flow control
	toptions.c_cflag &= ~CRTSCTS;

	toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
	toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

	toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
	toptions.c_oflag &= ~OPOST; // make raw

	/* see: http://unixwiz.net/techtips/termios-vmin-vtime.html */
	// Set read-timeout for reading each byte
	toptions.c_cc[VTIME] = WAIT_TIME;
	
	if( tcsetattr(fd, TCSANOW, &toptions) < 0) {
		perror("serialport_init: Couldn't set term attributes");
		return -1;
	}

	return fd;
}
	

	
	
	
	
	
	
	
	
	
