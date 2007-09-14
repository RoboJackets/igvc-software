#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <sys/ioctl.h>

#include "serial.h"
#include "exception.h"

using namespace std;

int ser_handle = -1;
const char *port;

const char *timeout_exception::what() const throw()
{
	return "Timeout reading data from target";
}

////////

void ser_init()
{
	struct termios attr;

	ser_handle = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
	if (ser_handle < 0)
		throw errno_exception("Can't open port");

	tcgetattr(ser_handle, &attr);
	cfmakeraw(&attr);
	cfsetospeed(&attr, B115200);
	tcflush(ser_handle, TCIOFLUSH);
	tcsetattr(ser_handle, TCSANOW, &attr);
}

void ser_shutdown()
{
	if (ser_handle >= 0)
		close(ser_handle);
}

void ser_send(unsigned char ch)
{
	write(ser_handle, &ch, 1);
}

unsigned char ser_receive()
{
	unsigned char ch;

	if (read(ser_handle, &ch, 1) != 1)
		throw errno_exception("Can't read from port");

	return ch;
}

unsigned char ser_receive_timed()
{
	unsigned char ch;

	//FIXME - Timeout
	if (read(ser_handle, &ch, 1) != 1)
		throw errno_exception("Can't read from port");

	return ch;
}

void ser_dtr(int state)
{
	int bits;

	ioctl(ser_handle, TIOCMGET, &bits);

	if (state)
		bits |= TIOCM_DTR;
	else bits &= ~TIOCM_DTR;

	ioctl(ser_handle, TIOCMSET, &bits);
}

