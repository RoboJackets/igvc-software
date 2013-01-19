#include <string.h>

#include "ASIOSerialPort.h"
#include <iostream>
#include "OSMC_driver.hpp"

using namespace std;

ASIOSerialPort arduinoLeft("/dev/arduinoLeft", 9600);
ASIOSerialPort arduinoRight("/dev/arduinoRight", 9600);
ASIOSerialPort arduinoEncoder("/dev/arduinoEncoder", 9600);

//TODO: Need to be checked
const byte OSMC_driver::maxPwm = 130;
const byte OSMC_driver::minPwm = 30;

OSMC_driver::~OSMC_driver()
{
	stopMotors();
	arduinoRight.close();
	arduinoLeft.close();
}

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

void OSMC_driver::setPwm(byte pwm, byte dir)	//writes pwm to arduinos setting both motors to the same pwm and direction (0 = forwards, 1 = backwards)
{
	checkPwm(pwm, dir);
	arduinoLeft.write("W");
	arduinoRight.write("W");
	arduinoLeft.write(dir);
	arduinoRight.write(dir);
	arduinoLeft.write(pwm);
	arduinoRight.write(pwm);
}

void OSMC_driver::setMotorsPwm(byte pwmLeft, byte dirLeft, byte pwmRight, byte dirRight)		//writes pwmLeft and pwmRight to respective arduinos
{
	checkPwm(pwmLeft, pwmRight);
	arduinoLeft.write("W");
	arduinoRight.write("W");
	arduinoLeft.write(dirLeft);
	arduinoRight.write(dirRight);
	arduinoLeft.write(pwmLeft);
	arduinoRight.write(pwmRight);
}

void OSMC_driver::goTurn(int degree, byte dir)
{

}

void OSMC_driver::goForward(double dist, byte pwm, byte dir)
{
    checkPwm(pwm, dir);
    arduinoLeft.write("WD");
    arduinoLeft.write(dist);
    arduinoLeft.write(dir);
    arduinoLeft.write(pwm);
    arduinoRight.write("WD");
    arduinoRight.write(dist);
    arduinoRight.write(dir);
    arduinoRight.write(pwm);
    while(arduinoRight.readln() == "!" && arduinoLeft.readln() == "!")
    {
        break;
    }
}

float OSMC_driver::readEncoder()
{
    arduinoEncoder.write("R");
    float dist = arduinoEncoder.readln();
    return dist;
}

void OSMC_driver::encoderLoop(float totalDist)
{
    dist = readEncoder();
    while (dist<totalDist)
    {
        dist = readEncoder();
    }
    stopMotors();
}

void OSMC_driver::goForwardOld(float totalDist, byte pwm, byte dir)
{
    checkPwm(pwm, dir);
    arduinoLeft.write("WS");
    arduinoLeft.write(dir);
    arduinoLeft.write(pwm);
    arduinoRight.write("WS");
    arduinoRight.write(dir);
    arduinoRight.write(pwm);
    encoderLoop(totalDist);

}

void OSMC_driver::stopMotors()		//Stops the motors
{
	arduinoLeft.write("S");
	arduinoRight.write("S");
}

void OSMC_driver::checkPwm(byte pwm, byte dir)	//checks that the pwm is within the minPwm and maxPwm
{
	if (pwm < minPwm)
	{
		setPwm(minPwm, dir);
	}
	else if (pwm > maxPwm)
	{
		setPwm(maxPwm, dir);
	}
	else
	{
		return;
	}
}

void OSMC_driver::checkPwm(byte pwmLeft, byte pwmRight)	//checks that the pwm for the left and right are within the minPwm and maxPwm
{
	if (pwmLeft < minPwm or pwmRight< minPwm)
	{
		setPwm(minPwm, 1);
	}
	else if (pwmLeft > maxPwm or pwmRight > maxPwm)
	{
		setPwm(maxPwm, 1);
	}
	else
	{
		return;
	}
}

