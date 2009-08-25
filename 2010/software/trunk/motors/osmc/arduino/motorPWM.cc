#include "motorPWM.hpp"

void setLeftMotorDutyCycle(char ldc)
{
	analogWrite(leftSpeedPin, ldc);
}

void setRightMotorDutyCycle(char rdc)
{
	analogWrite(rightSpeedPin, rdc);
}

void setPWMFreq()
{
	TCCR2B = TCCR2B & 0b11111000 | 0x03;//pin 11/3 @ 976.5625Hz

	TCCR0B = TCCR0B & 0b11111000 | 0x03;//pins 5/6 @ 976.5625Hz
}

