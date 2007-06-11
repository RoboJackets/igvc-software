#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdlib.h>
#include "rjte.h"
#include "comm.h"
#include "joystick.h"
#include "types.h"

#define HEARTBEAT_COUNT	50

//TODO:
// change default safetyMode
// fix button debounce count
// fix dead zone
// scale motorspeeds (pwm output)
// add uart timeout
// document everything -- note where changes in parameters will break certain functions (i.e. ACD left shift)

MOTORSTATES_T motorStates;
uint8_t lastButton1State;
uint8_t volatile controlMode;
uint8_t volatile safetyMode;
uint8_t volatile heartbeatCounter;

/* This is called when INT0 (PROG) is on the rising edge of a signal */
ISR(INT0_vect)
{
	uint8_t error = 0;
	if(((controlMode == AUTONOMOUS_MODE) || (controlMode == NAVIGATION_MODE)) && (safetyMode == DRIVE_MODE))
	{
		/* Read the motor speeds and other control data sent through the serial port */
		MOTORSTATES_T tempMS;
		if(!(error = get_speed_from_uart(&tempMS)))
		{
			heartbeatCounter = HEARTBEAT_COUNT;
			motorStates = tempMS;
			set_motors(motorStates);
		}
	}

	/* The speed and other control data is sent back in all modes */
	send_speed(motorStates, (controlMode | safetyMode), error);
}

/* This is called when timer0 overflows. */
ISR(TIMER0_OVF_vect)
{
	debounce_buttons();
	
	/* Process button and switches */
	if(CONTROL_MODE_SWITCH1 == PRESSED)
	{
		if(CONTROL_MODE_SWITCH2 == PRESSED)
		{
			LED1 = ON;
			LED2 = OFF;
			controlMode = AUTONOMOUS_MODE;
		}
		else
		{
			LED1 = OFF;
			LED2 = ON;
			controlMode = NAVIGATION_MODE;
		}
	}
	else
	{
		LED1 = ON;
		LED2 = ON;
		controlMode = JOYSTICK_MODE;
	}
	#if 0
	if(BUTTON1 == RELEASED)
	{
		lastButton1State = RELEASED;
	}
	else if(BUTTON1 == PRESSED)
	{
		if(lastButton1State == RELEASED)
		{
			if(controlMode == JOYSTICK_MODE)
			{
				controlMode = AUTONOMOUS_MODE;
				LED1 = OFF;
				LED2 = ON;
			}

			else if(controlMode == AUTONOMOUS_MODE)
			{
				controlMode = JOYSTICK_MODE;
				LED1 = ON;
				LED2 = ON;
			}
			/*CONTROLBUTTON
			controlMode = NAVIGATION_MODE;
			LED1 = ON;
			LED2 = OFF;*/
		}
		lastButton1State = PRESSED;
	}
	#endif


	#if 0
	if((STOP_BUTTON == PRESSED) || (JOYSTICK_SAFTEY_BUTTON == PRESSED))
	{
		LED1 = OFF;
		LED2 = OFF;
		safetyMode = STOP_MODE;
		/* Stop the motors */
		motorStates = (MOTORSTATES_T){0, 0, FORWARD, FORWARD};
		set_motors(motorStates);
	}
	else
	{
		safetyMode = DRIVE_MODE;
	}
	#endif

	if(controlMode == JOYSTICK_MODE) //&& (safetyMode == DRIVE_MODE))
	{
		/* Update the motor states based on the joystick position */
		motorStates = get_speed_from_joystick();
		set_motors(motorStates);
		heartbeatCounter = HEARTBEAT_COUNT;
	}
	heartbeatCounter--;
	if(heartbeatCounter)
	{
		motorStates = (MOTORSTATES_T){0, 0, FORWARD, FORWARD};
		set_motors(motorStates);
	}
}

int main(void)
{
	/* Initalize variables and setup hardware */
	lastButton1State = RELEASED;
	controlMode = JOYSTICK_MODE;
	safetyMode = DRIVE_MODE;
	motorStates = (MOTORSTATES_T){0, 0, FORWARD, FORWARD};
	heartbeatCounter = HEARTBEAT_COUNT;
	hardware_init();

	for(;;)
	{
		/* Sleep until recieving an interrupt */
		sleep_mode();
	}
	return(0);
}
