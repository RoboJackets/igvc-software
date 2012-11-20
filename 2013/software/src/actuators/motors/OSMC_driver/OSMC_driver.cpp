#include <string.h>

#include "ASIOSerialPort.h"
#include <iostream>
#include "OSMC_driver.hpp"

using namespace std;

ASIOSerialPort arduinoLeft("/dev/arduinoLeft", 9600);
ASIOSerialPort arduinoRight("/dev/arduinoRight", 9600);

//TODO: Need to be checked
const int OSMC_driver::maxPwm = 130;
const int OSMC_driver::minPwm = 30;

bool OSMC_driver::arduinoCheck()	//Checks we are connected and able to send and receive data from the arduinos
{
	std::cout << "Testing serial API..." << std::endl;
	arduinoLeft.write("T");
	arduinoRight.write("T");
	sleep(0.01);
	if (arduinoRight.readln() == "T" and arduinoLeft.readln() == "T")
	{
		std::cout<<"Successfully read from Arduinos" << std::endl;
		return true;
	}
	else
	{
		std::cout<<"Failed to read from Arduinos"<<std::endl;
		arduinoLeft.close();
		arduinoRight.close();
		return false;
	}
}

void OSMC_driver::setPwm(int pwm)	//writes pwm to arduinos setting both motors to the same pwm
{
	checkPwm(pwm);
	arduinoLeft.write("W");
	arduinoLeft.write(pwm);
	arduinoRight.write("W");
	arduinoRight.write(pwm);
}

void OSMC_driver::setMotorsPwm(int pwmLeft, int pwmRight)		//writes pwmLeft and pwmRight to respective arduinos
{
	checkPwm(pwmLeft, pwmRight);
	arduinoLeft.write("W");
	arduinoLeft.write(pwmLeft);
	arduinoRight.write("W");
	arduinoRight.write(pwmRight);
}

void OSMC_driver::stopMotors()		//Stops the motors
{
	arduinoLeft.write("S");
	arduinoRight.write("S");
}

void OSMC_driver::checkPwm(int pwm)
{
	if (pwm < minPwm)
	{
		setPwm(minPwm);
	}
	else if (pwm > maxPwm)
	{
		setPwm(maxPwm);
	}
	else
	{
		return;
	}
}

void OSMC_driver::checkPwm(int pwmLeft, int pwmRight)
{
	if (pwmLeft < minPwm or pwmRight< minPwm)
	{
		setPwm(minPwm);
	}
	else if (pwmLeft > maxPwm or pwmRight > maxPwm)
	{
		setPwm(maxPwm);
	}
	else
	{
		return;
	}
}

