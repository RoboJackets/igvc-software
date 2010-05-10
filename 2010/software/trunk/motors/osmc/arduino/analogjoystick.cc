#include "analogjoystick.hpp"

void setupJoystick()
{
	pinMode(joystickXADC, INPUT);
	pinMode(joystickXADC, INPUT);
	pinMode(joystickEnable, INPUT);
	digitalWrite(joystickEnable, HIGH);
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

	const int XMiddle = 1023 / 2;
	const int YMiddle = 1023 / 2;

	getJoystickReading(&xAxis, &yAxis);

	//shift 0
	xAxis -= XMiddle;
	yAxis -= YMiddle;

	//make x go the right dir
	//xAxis *= -1;

	/* Apply deadzones */
	if (abs(xAxis) < DEAD_ZONE)
	{ 
		xAxis = 0;
	}
	if (abs(yAxis) < DEAD_ZONE)
	{
		yAxis = 0;
	}

	/* Calculate drive outputs -- magic*/
	//int leftVelocity = .5*float(yAxis) + 2 * sign(xAxis) * sqrt(abs(xAxis));
	//int rightVelocity = .5*float(yAxis) - 2 * sign(xAxis) * sqrt(abs(xAxis));
	//int leftVelocity = yAxis + 3 * sign(xAxis) * sqrt(abs(xAxis));
	//int rightVelocity = yAxis - 3 * sign(xAxis) * sqrt(abs(xAxis));
	int leftVelocity;
	int rightVelocity;
	
	if ((yAxis > xAxis)&&(xAxis >= 0)) {
	  leftVelocity = 512;
	  rightVelocity = 512 - xAxis/2;
	}
	else if ((yAxis <= xAxis)&&(yAxis > 0)) {
	  leftVelocity = 256 + yAxis/2;
	  rightVelocity = yAxis - 256;
	}
	else if ((-yAxis < xAxis)&&(yAxis <= 0)) {
	  leftVelocity = 256 - yAxis;
	  rightVelocity = -256 - yAxis/2;
	}
	else if ((-yAxis >= xAxis)&&(xAxis > 0)) {
	  leftVelocity = -512 + xAxis/2;
	  rightVelocity = -512;
	}
	else if ((yAxis < xAxis)&&(xAxis <= 0)) {
	  leftVelocity = -512;
	  rightVelocity = -512 - xAxis/2;
	}
	else if ((yAxis >= xAxis)&&(yAxis < 0)) {
	  leftVelocity = -256 + yAxis/2;
	  rightVelocity = -256 + yAxis;
	}
	else if ((-yAxis < xAxis)&&(yAxis >= 0)) {
	  leftVelocity = -256 + yAxis;
	  rightVelocity = 256 + yAxis/2;
	}
	else if ((-yAxis >= xAxis)&&(xAxis < 0)) {
	  leftVelocity = 512 + xAxis/2;
	  rightVelocity = 512;
	}

	leftVelocity = float(leftVelocity/2)*abs(float(xAxis + yAxis)/float(1023 - abs(xAxis - yAxis)));
	rightVelocity = float(rightVelocity/2)*abs(float(xAxis + yAxis)/float(1023 - abs(xAxis - yAxis)));

	/* Limit drive velcocities to vaild values and convert them to drive speeds and directions */

	Serial.print("L/Rev: ");
	Serial.println(leftVelocity, DEC);

	Serial.print("R/Rev: ");
	Serial.println(rightVelocity, DEC);

	byte leftSpeed = min(abs(leftVelocity), 255);
	byte leftDir = (leftVelocity < 0) ? MC_MOTOR_REVERSE : MC_MOTOR_FORWARD;
	byte rightSpeed = min(abs(rightVelocity), 255);
	byte rightDir = (rightVelocity < 0) ? MC_MOTOR_REVERSE : MC_MOTOR_FORWARD;

	setLeftMotorDutyCycle(leftDir, leftSpeed);
	setRightMotorDutyCycle(rightDir, rightSpeed);

}
