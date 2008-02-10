#include "comm.h"

/* Various values for communicating with the computer */
#define ERROR_MASK			0x3c
#define ERROR_OFFSET			2
#define CONTROL_MODE_MASK		0xc0
#define CONTROL_MODE_OFFSET		6
#define LEFT_DIR_MASK			0x01
#define LEFT_DIR_OFFSET			0
#define RIGHT_DIR_MASK			0x02
#define RIGHT_DIR_OFFSET		1

//TODO: describe what is sent
inline void send_speed(MOTORSTATES_T motorStates, uint8_t controlMode, uint8_t error)
{
	uart_put((controlMode << CONTROL_MODE_OFFSET) | (error << ERROR_OFFSET) |
		(motorStates.leftDir << LEFT_DIR_OFFSET) |(motorStates.rightDir << RIGHT_DIR_OFFSET) ); // first byte
	uart_put(motorStates.leftSpeed); // second byte
	uart_put(motorStates.rightSpeed); // third byte
	return;
}

//TODO: describe what is recieved
inline uint8_t get_speed_from_uart(MOTORSTATES_T* motorStates)
{
	int16_t buff[3];

	int16_t i;
	for(i = 0; i < 3; i++)
	{
		/* Read each byte from the uart and stop if there are any errors */
		if((buff[i] = uart_get()) < 0)
		{
			LED3 = OFF;

			/* Return the negative of error sent by uart_get(), making it a normal UART_ERROR_T */
			return(-buff[i]);
		}
	}
	LED3 = ON;
	motorStates->leftDir = (buff[0] & LEFT_DIR_MASK) >> LEFT_DIR_OFFSET;
	motorStates->rightDir = (buff[0] & RIGHT_DIR_MASK) >> RIGHT_DIR_OFFSET;
	motorStates->leftSpeed = (uint8_t)buff[1];
	motorStates->rightSpeed = (uint8_t)buff[2];
	return(0);
}
