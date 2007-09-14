#include <stdio.h>
#include <stdlib.h>
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/ioctl.h>

int main(int argc, char *argv[])
{
	/* Open serial port */
	int fdSerialPort = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_DSYNC | O_RSYNC | O_NDELAY);
	if (fdSerialPort == -1) 
		printf("Unable to open /dev/ttyS0\n");

	/* Set the port back to blocking (waiting) behavior */
	fcntl(fdSerialPort, F_SETFL, 0);

	/* Get the current options for the port and store it in <options> */
	struct termios options;
	tcgetattr(fdSerialPort, &options);

	/* Configure the port */	 
	options.c_cflag |= (CREAD | CLOCAL); /* Enable the receiver and set local mode */
	cfsetispeed(&options, B115200); /* 115200 baud rate */
	cfsetospeed(&options, B115200); 
	options.c_cflag &= ~CSIZE; /* Set 8 data bits */
	options.c_cflag |= CS8;
	options.c_cflag &= ~PARENB; /* no parity */
	options.c_cflag &= ~CSTOPB; /* 1 stop bit */
	options.c_cc[VMIN]  = 0; /* Set timeout */
	options.c_cc[VTIME] = 10;
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); /* Set raw input and output */
	options.c_oflag &= ~OPOST;

    	/* Flush input and output buffers and then set the new options for the port */
	tcsetattr(fdSerialPort, TCSAFLUSH, &options);

	printf("Start recieving\n");
	unsigned char buff[3] = {0, 43, 28};
	unsigned char data[4];
	while(1)
	{
	#if 0
		write(fdSerialPort, buff, 3);
		
		/* Make sure the data has finished being written */
		tcdrain(fdSerialPort);

		/* Flush any old data in the read buffer */
		tcflush(fdSerialPort, TCIFLUSH);

		/* Set the DTR line high to intterupt the controller */
		int status;
		ioctl(fdSerialPort, TIOCMGET, &status);
		status |= TIOCM_DTR;
		ioctl(fdSerialPort, TIOCMSET, &status);		

		/* Set the DTR line back to low */
		ioctl(fdSerialPort, TIOCMGET, &status);
		status &= ~TIOCM_DTR;
		ioctl(fdSerialPort, TIOCMSET, &status);


		int bytes;
		if(ioctl(fdSerialPort, FIONREAD, &bytes))
		{
			printf("It no worky\n");
		}
		else
		{
			printf("%d bytes avaiable\n", bytes);
		}*/
		#endif
		/*while(read(fdSerialPort, data, 1) > 0)
		{
			printf("-%d- ", (int)buff[0]);
		}*/
		//if(read(fdSerialPort, data, 3) == 3)
		//{
			read(fdSerialPort, data, 1);
			printf("Recieved: %d \n", data[0]);
			printf("\n");
		/*}
		else
		{
			printf("read no worky\n");
		}*/

		//sleep(1);


	}

	close(fdSerialPort);

  return EXIT_SUCCESS;
}
