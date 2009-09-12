#include "motorPWM.hpp"

void setLeftMotorDutyCycle(byte dir, byte ldc)
{
	if(dir == MC_MOTOR_FORWARD)
	{
		digitalWrite(leftDirectionPin, HIGH);
		analogWrite(leftSpeedPin, 255 - ldc);
	}
	else
	{
		digitalWrite(leftDirectionPin, LOW);
		analogWrite(leftSpeedPin, ldc);
	}
}

void setRightMotorDutyCycle(byte dir, byte rdc)
{
	if(dir == MC_MOTOR_FORWARD)
	{
		digitalWrite(rightDirectionPin, HIGH);
		analogWrite(rightSpeedPin, 255 - rdc);
	}
	else
	{
		digitalWrite(rightDirectionPin, LOW);
		analogWrite(rightSpeedPin, rdc);

	}
}

/*
void setLeftMotorDutyCycle(byte dir, byte ldc)
{
	if(dir == MC_MOTOR_FORWARD)
	{
		digitalWrite(leftDirectionPin, LOW);
		analogWrite(leftSpeedPin, ldc);
	}
	else
	{
		digitalWrite(leftDirectionPin, HIGH);
		analogWrite(leftSpeedPin, 255 - ldc);
	}
}

void setRightMotorDutyCycle(byte dir, byte rdc)
{
	if(dir == MC_MOTOR_FORWARD)
	{
		digitalWrite(rightDirectionPin, LOW);
		analogWrite(rightSpeedPin, rdc);
	}
	else
	{
		digitalWrite(rightDirectionPin, HIGH);
		analogWrite(rightSpeedPin, 255 - rdc);
	}
}
*/
void setPWMFreq()
{
	TCCR2B = TCCR2B & 0xF8 | 0x03;//pin 11/3 @ 976.5625Hz

	TCCR0B = TCCR0B & 0xF8 | 0x03;//pins 5/6 @ 976.5625Hz
}

