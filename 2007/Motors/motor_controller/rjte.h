#ifndef _RJTE_H_
#define _RJTE_H_
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "types.h"

/* CPU clock frequency - must be defined before util/delay.h is included */
#define F_CPU 14.7456E6
#include <util/delay.h>

/* Pin assignments:
 *
 * Port B:
 * Bit	Dir	Pull-up	Description
 * 0	O	-	Motor A direction
 * 1	O	-	Motor A speed (OC1A)
 * 2	O	-	Motor B speed (OC1B)
 * 3	O	-	Motor B direction
 * 4	I	U	Button 2 (SW3)
 * 5	I	U	Button 1 (SW2)
 * 6	I	U	Unused
 * 7	I	U	Unused
 *
 * Port C:
 * Bit	Dir	Pull-up	Description
 * 0	I	U	Digial Input 0
 * 1	I	U	Digial Input 1
 * 2	I	U	Digial Input 2
 * 3	I	U	Digial Input 3
 * 4	I	U	Digial Input 4
 * 5	I	U	Digial Input 5
 * 6	I	-	Reset (not usable as I/O)
 *
 * Port D:
 * Bit	Dir	Pull-up	Description
 * 0	I	U	Serial RXD
 * 1	O	-	Serial TXD
 * 2	I	U	Serial DTR (PROG)
 * 3	I	U	Motor error flag (INT1)
 * 4	I	U	Joystick button
 * 5	O	-	LED 1 (D2), active low
 * 6	O	-	LED 2 (D3), active low
 * 7	O	-	LED 3 (D4), active low
 */

/* Call this only once as the first thing in main() */
inline void hardware_init(void);

/* The LEDs are connected between the I/O pin and Vcc, so 0 turns them on. */
#define ON		0
#define OFF		1

#define LED1		IO_BYTE(&PORTD).bit_5		//0x32
#define LED2		IO_BYTE(&PORTD).bit_6
#define LED3		IO_BYTE(&PORTD).bit_7

/* Raw button access */
uint8_t button1RawValue(void);
uint8_t button2RawValue(void);
uint8_t digIn0RawValue(void);
uint8_t digIn1RawValue(void);
uint8_t digIn2RawValue(void);
uint8_t digIn3RawValue(void);


#define NUMBER_OF_BUTTONS	6

/* Debounced button states */
typedef struct
{
	uint8_t value:1;
	uint8_t count:7;
	uint8_t (*rawValue)(void);
} button_t;

button_t buttons[NUMBER_OF_BUTTONS];


/* Use these to simplfy reading the values of the debounced buttons */
#define BUTTON1			(buttons[0].value)
#define BUTTON2			(buttons[1].value)
#define CONTROL_MODE_SWITCH1	(buttons[2].value)
#define CONTROL_MODE_SWITCH2	(buttons[4].value)
//#define STOP_BUTTON		(buttons[4].value)
#define JOYSTICK_SAFTEY_BUTTON	(buttons[5].value)

/* Possible button values */
#define PRESSED  0
#define RELEASED 1

/* This should be called regularly by a timer to debounce the buttons.
 * Only button releases are debounced.
 */
inline void debounce_buttons(void);

/* The number of timer1 overflows required that a button be up
 * before it is considered to have been released.
 */
#define DEBOUNCE_COUNT		5

/* Use this to set the motor states */
void set_motors(MOTORSTATES_T motorStates);

/* Motor directions */
#define FORWARD		0
#define REVERSE		1

/* Use these functions to send and recieve bytes over the serial port.
 *	If you want something to send strings, avr-libc has a 
 *	reduced stdio libray that you can use instead of this.
 */
void uart_put(uint8_t c);
int16_t uart_get(void);
void uart_flush(void);

/* uart_get will return the negative of these if there is an error */
enum{
	FRAME_ERROR = 1,
	DATA_OVERRUN_ERROR = 2,
	PARITY_ERROR = 4,
	TIMEOUT_ERROR = 8
} UART_ERROR_T;

/* Use this to read values from an ADC channel */
uint8_t adc_value(uint8_t channel);

#endif /* _RJTE_H_ */
