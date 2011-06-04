#include "motorPWM.hpp"


#if 1
void setLeftMotorDutyCycle(byte dir, byte ldc)
{
	byte scaledldc = float(ldc) * leftScale;

	if(scaledldc > 200) scaledldc = 200;

	if(dir == MC_MOTOR_FORWARD)
	{
		//Serial.print("Motor DBG - L/Fwd: ");
		//Serial.print(scaledldc, DEC);

		digitalWrite(leftDirectionPin, HIGH);
		analogWrite(leftSpeedPin, 255 - scaledldc);
	}
	else
	{
		//Serial.print("Motor DBG - L/Rev: ");
		//Serial.print(scaledldc, DEC);

		digitalWrite(leftDirectionPin, LOW);
		analogWrite(leftSpeedPin, scaledldc);
	}
}

void setRightMotorDutyCycle(byte dir, byte rdc)
{
	byte scaledrdc = float(rdc) * rightScale;

	if(scaledrdc > 200) scaledrdc = 200;

	if(dir == MC_MOTOR_FORWARD)
	{
		//Serial.print("Motor DBG - R/Fwd: ");
		//Serial.print(scaledrdc, DEC);

		digitalWrite(rightDirectionPin, LOW);
		analogWrite(rightSpeedPin, scaledrdc);
	}
	else
	{
		//Serial.print("Motor DBG - R/Rev: ");
		//Serial.print(scaledrdc, DEC);
		digitalWrite(rightDirectionPin, HIGH);
		analogWrite(rightSpeedPin, 255 - scaledrdc);


	}
}
#else
void setLeftMotorDutyCycle(byte dir, byte ldc)
{
	byte scaledldc = float(ldc) * leftScale;

	if(dir == MC_MOTOR_FORWARD)
	{
		//Serial.print("Motor DBG - L/Fwd: ");
		//Serial.print(scaledldc, DEC);

		digitalWrite(leftDirectionPin, LOW);
		analogWrite(leftSpeedPin, scaledldc);
	}
	else
	{
		//Serial.print("Motor DBG - L/Rev: ");
		//Serial.print(scaledldc, DEC);
		digitalWrite(leftDirectionPin, HIGH);
		analogWrite(leftSpeedPin, 255 - scaledldc);

	}
}

void setRightMotorDutyCycle(byte dir, byte rdc)
{
	byte scaledrdc = float(rdc) * rightScale;

	if(dir == MC_MOTOR_FORWARD)
	{
		//Serial.print("Motor DBG - R/Fwd: ");
		//Serial.print(scaledrdc, DEC);

		digitalWrite(rightDirectionPin, HIGH);
		analogWrite(rightSpeedPin, scaledrdc);
	}
	else
	{
		//Serial.print("Motor DBG - R/Rev: ");
		//Serial.print(scaledrdc, DEC);
		digitalWrite(rightDirectionPin, LOW);
		analogWrite(rightSpeedPin, 255 - scaledrdc);
	}
}
#endif
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
	//TCCR2B = (TCCR2B & 0xF8) | 0x02;//pin 11/3 @ 2x976.5625Hz
	//TCCR0B = (TCCR0B & 0xF8) | 0x02;//pins 5/6 @ 2x976.5625Hz
}

