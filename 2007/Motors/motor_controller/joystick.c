#include "joystick.h"

#define DEAD_ZONE		10
#define JOYSTICK_OFFSET		128

#define min(a,b) (((a)<(b)) ? (a) : (b))

/* Acesses the H.O. byte values of the two ADC channels X_AXIS and Y_AXIS
 *	shifts them by the value in JOYSTICK_OFFSET and converts them to motor
 *	speeds and directions based on the selected drive mode.
 */
inline MOTORSTATES_T get_speed_from_joystick(void)
{
	int16_t xAxis, yAxis;	// -128 to 127
	int16_t leftVelocity, rightVelocity;	// -256 to 256
	int8_t trash;

	/* Retrive the position of the joystick - hacked solution to get rid of bad values */
	trash = adc_value(JOYSTICK_X_AXIS);
	xAxis = (int16_t)adc_value(JOYSTICK_X_AXIS) - JOYSTICK_OFFSET;

	trash = adc_value(JOYSTICK_Y_AXIS);
	yAxis = (int16_t)adc_value(JOYSTICK_Y_AXIS) - JOYSTICK_OFFSET;

	/* Apply deadzones */
	if (abs(xAxis) < DEAD_ZONE){ xAxis = 0; }
	if (abs(yAxis) < DEAD_ZONE){ yAxis = 0; }

	/* Calculate drive outputs */
	leftVelocity = yAxis + xAxis;
	rightVelocity = yAxis - xAxis;

	/* Limit drive velcocities to vaild values and convert them to drive speeds and directions */
	MOTORSTATES_T motorStates;
	motorStates.leftSpeed = min(abs(leftVelocity), 255);
	motorStates.leftDir = (leftVelocity < 0) ? REVERSE : FORWARD;
	motorStates.rightSpeed = min(abs(rightVelocity), 255);
	motorStates.rightDir = (rightVelocity < 0) ? REVERSE : FORWARD;

	return(motorStates);
}
