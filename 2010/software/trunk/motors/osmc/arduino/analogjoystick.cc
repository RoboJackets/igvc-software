#include "analogjoystick.hpp"

void setupJoystick()
{
	pinMode(joystickXADC, INPUT);
	pinMode(joystickXADC, INPUT);
	pinMode(joystickEnable, INPUT);
}
/*
void disableJoystick()
{
	digitalWrite(joystickEnable, LOW);
}
*/
void getJoystickReading(int* X, int* Y)
{
	*X = analogRead(joystickXADC);
	*Y = analogRead(joystickYADC);
}

int sign(int n)
{
	if(n > 0)
	{
		return 1;
	}
	return -1;
}

/* Get joystick position and turn it into motor velocities */
// 
//   +y
//  -x +x
//    -y
//
void joystickSetMotors()
{
	int xAxis = 0;
	int yAxis = 0;

	getJoystickReading(&xAxis, &yAxis);

	/* Apply deadzones */
	if (abs(xAxis) < DEAD_ZONE){ xAxis = 0; }
	if (abs(yAxis) < DEAD_ZONE){ yAxis = 0; }

	/* Calculate drive outputs -- magic*/
	int leftVelocity = yAxis + 3 * sign(xAxis) * sqrt(abs(xAxis));
	int rightVelocity = yAxis - 3 * sign(xAxis) * sqrt(abs(xAxis));

	/* Limit drive velcocities to vaild values and convert them to drive speeds and directions */

	int leftSpeed = min(abs(leftVelocity), 255);
	int leftDir = (leftVelocity < 0) ? MC_MOTOR_REVERSE : MC_MOTOR_FORWARD;
	int rightSpeed = min(abs(rightVelocity), 255);
	int rightDir = (rightVelocity < 0) ? MC_MOTOR_REVERSE : MC_MOTOR_FORWARD;

	setLeftMotorDutyCycle(leftDir, leftSpeed);
	setRightMotorDutyCycle(rightDir, rightSpeed);

}
