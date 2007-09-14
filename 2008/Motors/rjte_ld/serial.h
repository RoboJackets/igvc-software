#ifndef _SERIAL_H_
#define _SERIAL_H_

#include <exception>

#define DEFAULT_PORT	"/dev/ttyUSB0"

extern const char *port;

class timeout_exception: public std::exception
{
public:
	const char *what() const throw();
};

/* Opens the serial port */
void ser_init(void);

/* Sends one character to the serial port */
void ser_send(unsigned char ch);

/* Receives one character and returns it. */
unsigned char ser_receive(void);

/* Receives one character and returns it. */
unsigned char ser_receive_timed(void);

/* Sets the state of the DTR line */
void ser_dtr(int state);

/* Closes the serial port */
void ser_shutdown(void);

#endif /* _SERIAL_H_ */

