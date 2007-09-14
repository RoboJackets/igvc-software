#include "rjte.h"

/* intializes all of the hardware */
inline void hardware_init(void)
{
	/* Set up I/O ports 
	 *
	 * If DDx is written to ZERO, Pxn is configured as an INPUT pin.
	 * If DDx is written to ONE, Pxn is configured as an OUTPUT pin.
	 *
	 * If PORTxn is written to ONE when the pin is configured as an INPUT pin,
	 *		the PULL-UP RESISTOR is activated.
	 * If PORTxn is written to ONE when the pin is configured as an OUTPUT pin,
	 *		the port pin is DRIVEN HIGH (one).
	 */
	DDRB = 0x0f;
	DDRC = 0x00;
	DDRD = 0xe2;
	PORTB = 0xf0;
	PORTC = 0x7f;
	PORTD = 0x1d;

	/* Stop the motors */
	//set_motors( (MOTORSTATES_T){0, 0, FORWARD, FORWARD} );

	/* Set up the ADC*/
	ADMUX = (0<<REFS1)|(0<<REFS0) | (1<<ADLAR) | (0<<MUX3)|(1<<MUX2)|(1<<MUX1)|(0<<MUX0);
	ADCSRA = (1<<ADEN)| (1<<ADSC) | (0<<ADFR) | (0<<ADIF)|(0<<ADIE) | (0<<ADPS2)|(0<<ADPS1)|(0<<ADPS0);

	/* Set up the UART */
	UBRRH = 0;				/* Set 115200 baud, U2X=0 */
	UBRRL = 7;
	UCSRA = (1 << TXC);			/* Clear TXC, turn off things we don't use */
	UCSRB = (1 << TXEN) | (1 << RXEN);	/* Turn on TX and RX */
	UCSRC = (1 << URSEL) | (3 << UCSZ0);	/* Asynchronous mode, 8-bit characters, no parity, 1 stop bit */
	MCUCR = (0<<SE)|(0<<SM2)|(0<<SM1)|(0<<SM0)|(0<<ISC11)|(0<<ISC10)|(1<<ISC01)|(1<<ISC00); /* Set PROG to interrupt on rising edge (but is not enabled yet)*/

	/* Set up timer1 for PWM */
	TCCR1A = 0xa1; //(1<<COM1A1)|(0<<COM1A0)|(1<<COM1B1)|(0<<COM1B0)|(0<<FOC1A)|(0<<FOC1B)|(0<<WGM11)|(1<<WGM10);
	TCCR1B = 0x01; //(0<<ICNC1)|(0<<ICES1)|(0<<WGM13)|(0<<WGM12)|(0<<CS12)|(0<<CS11)|(1<<CS10);  /* prescaler of 1 = 24KHz pwm @ 14.7456MHz */ //

	/* Set up timer0 for polling the joystick and debouncing the buttons */
	TCCR0 = 0x05;  /* prescaler of 1024 = 56 overflows/sec @ 14.7456MHz */

	/* Enable the timer0, uart interrupts */
	TIMSK = (1<<TOIE0);
	GICR |= (1<<INT0);

	/* Setup the buttons for debouncing */
	buttons[0] = (button_t){RELEASED, DEBOUNCE_COUNT, button1RawValue};
	buttons[1] = (button_t){RELEASED, DEBOUNCE_COUNT, button2RawValue};
	buttons[2] = (button_t){RELEASED, DEBOUNCE_COUNT, digIn0RawValue};
	buttons[3] = (button_t){RELEASED, DEBOUNCE_COUNT, digIn1RawValue};
	buttons[4] = (button_t){RELEASED, DEBOUNCE_COUNT, digIn2RawValue};
	buttons[5] = (button_t){RELEASED, DEBOUNCE_COUNT, digIn3RawValue};

	/* Global interrupt enable */
	sei();

	return;
}

/* Motor direction outputs */
#define MOTOR_DIR_A	IO_BYTE(&PORTB).bit_0             // FIXME: make sure this isn't PINB or 0x38
#define MOTOR_DIR_B	IO_BYTE(&PORTB).bit_3

void set_motors(MOTORSTATES_T motorStates)
{
	/* Set the Timer1 Output Compare Registers, which the period of the PWM that is sent to the motors
	 *	If the direction is forward, the PWM is inverted.
	 */
	OCR1AH = 0;
	OCR1AL = (motorStates.leftDir == FORWARD) ? (motorStates.leftSpeed) : (255 - motorStates.leftSpeed);
	OCR1BH = 0;
	OCR1BL = (motorStates.rightDir == FORWARD) ? (motorStates.rightSpeed) : (255 - motorStates.rightSpeed);

	/* Set the enable line to flip the H-bridge to the correct direction */
	MOTOR_DIR_A = motorStates.leftDir;
	MOTOR_DIR_B = motorStates.rightDir;
}

void uart_put(uint8_t data)
{
	/* Wait for the transmit buffer to empty */
	loop_until_bit_is_set(UCSRA, UDRE);

	/* Put data into the send buffer */
	UDR = data;

	return;
}

int16_t uart_get(void)
{
	/* Check for any receive errors */
	uint8_t error = UCSRA & ((1<<FE)|(1<<DOR)|(1<<PE));
	if(error)
	{
		uart_flush();
		int16_t returnError = 0;
		if(error & (1<<FE))
			returnError |= FRAME_ERROR;
		if(error & (1<<DOR))
			returnError |= DATA_OVERRUN_ERROR;
		if(error & (1<<PE))
			returnError |= PARITY_ERROR;
		return(-returnError);
	}

	/* Wait for data to be received or a timeout */
	/* Return data in receive buffer */
	if(bit_is_set(UCSRA, RXC))
		return(UDR);				// FIXME: add a timeout here if needed
	else /* timeout */
	{
		return(-TIMEOUT_ERROR);
	}
}

void uart_flush(void)
{
	uint8_t dummy;

	/* Read every byte in the receive buffer */
	while (UCSRA & (1<<RXC)) dummy = UDR;
}

uint8_t adc_value(uint8_t channel)
{
	/* Chage the ADC to sample from <channel> */
	ADMUX = (ADMUX & 0xf0) | (channel & 0x0f);

	/* Start an A to D conversion */
	ADCSRA |= (1<<ADSC);

	/* Wait until the conversion is complete */
	loop_until_bit_is_clear(ADCSRA, ADSC);
	
	/* Return the result of the conversion */
	return(ADCH);
}

/* These define how to access a button or switch */
uint8_t button1RawValue(void){ return( IO_BYTE(&PINB).bit_5); }
uint8_t button2RawValue(void){ return( IO_BYTE(&PINB).bit_4 ); }
uint8_t digIn0RawValue(void){ return( IO_BYTE(&PINC).bit_0 ); }
uint8_t digIn1RawValue(void){ return( IO_BYTE(&PINC).bit_1 ); }
uint8_t digIn2RawValue(void){ return( IO_BYTE(&PINC).bit_2 ); }
uint8_t digIn3RawValue(void){ return( IO_BYTE(&PINC).bit_3 ); }


inline void debounce_buttons(void)
{
	uint8_t i;
	for(i = 0; i < NUMBER_OF_BUTTONS; i++)
	{
		if(buttons[i].rawValue() == PRESSED)
		{
			/* Button is pressed: indicate immediately */
			buttons[i].value = PRESSED;
			buttons[i].count = DEBOUNCE_COUNT;
		}
		else
		{
			/* Button is released: Wait for several contiguous samples before changing the value */
			if(buttons[i].count)
			{
				buttons[i].count--;
			}
			else
			{
				buttons[i].value = RELEASED;
			}
		}
	}
}
