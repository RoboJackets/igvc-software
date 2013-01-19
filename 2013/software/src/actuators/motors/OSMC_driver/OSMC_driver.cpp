#include <string.h>

#include "serial/ASIOSerialPort.h"
#include <iostream>
#include "actuators/motors/OSMC_driver/OSMC_driver.hpp"

using namespace std;

ASIOSerialPort arduinoLeft("/dev/igvc_2012_left_motor_shield", 9600);
ASIOSerialPort arduinoRight("/dev/igvc_2012_right_motor_shield", 9600);
ASIOSerialPort arduinoEncoder("/dev/igvc_2012_right_encoder_shield", 9600);

//TODO: Need to be checked
const char OSMC_driver::maxPwm = 130;
const char OSMC_driver::minPwm = 30;

OSMC_driver::~OSMC_driver()
{
	stopMotors();
	arduinoRight.close();
	arduinoLeft.close();
	arduinoEncoder.close();
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

void OSMC_driver::setPwm(char pwm, char dir)	//writes pwm to arduinos setting both motors to the same pwm and direction (0 = forwards, 1 = backwards)
{
	checkPwm(pwm, dir);
	arduinoLeft.write("WS" + dir + pwm);
	arduinoRight.write("WS"+dir+pwm);
}

void OSMC_driver::setMotorsPwm(char pwmLeft, char dirLeft, char pwmRight, char dirRight)		//writes pwmLeft and pwmRight to respective arduinos
{
	checkPwm2(pwmLeft, pwmRight);
	arduinoLeft.write("W" + dirLeft + pwmLeft);
	arduinoRight.write("W"+dirRight+pwmRight);
}

void OSMC_driver::goTurn(int degree, char dir)
{

}

void OSMC_driver::goForward(double dist, char pwm, char dir)
{
    checkPwm(pwm, dir);

    std::ostringstream left;
    left << dist << dir << pwm;
//    std::cout<<s.str().c_str()<<endl;

    arduinoLeft.write("WD");
    arduinoLeft.write(left.str().c_str());

    std::ostringstream right;
    right << dist << dir << pwm;

    arduinoRight.write("WD");
    arduinoRight.write(right.str().c_str());

    while(arduinoRight.readln() == "!" && arduinoLeft.readln() == "!")
    {
        break;
    }
}

float OSMC_driver::readEncoder()
{
    arduinoEncoder.write("R");
    std::string dist1 = arduinoEncoder.readln();
    double dist = ::atof(dist1.c_str());
    return dist;
}

void OSMC_driver::encoderLoop(float totalDist)
{
    double dist = readEncoder();
    while (dist<totalDist)
    {
        dist = readEncoder();
    }
    stopMotors();
}

void OSMC_driver::goForwardOld(float totalDist, char pwm, char dir)
{
    checkPwm(pwm, dir);

    arduinoLeft.write("WS"+dir+pwm);
    arduinoRight.write("WS"+dir+pwm);

    encoderLoop(totalDist);

}

void OSMC_driver::stopMotors()		//Stops the motors
{
	arduinoLeft.write("S");
	arduinoRight.write("S");
}

void OSMC_driver::checkPwm(char pwm, char dir)	//checks that the pwm is within the minPwm and maxPwm
{
	if (pwm < minPwm)
	{
		setPwm(minPwm, dir);
	}
	else if (pwm > maxPwm)
	{
		setPwm(maxPwm, dir);
	}
}

void OSMC_driver::checkPwm2(char pwmLeft, char pwmRight)	//checks that the pwm for the left and right are within the minPwm and maxPwm
{
	if (pwmLeft < minPwm or pwmRight< minPwm)
	{
		setPwm(minPwm, 1);
	}
	else if (pwmLeft > maxPwm or pwmRight > maxPwm)
	{
		setPwm(maxPwm, 1);
	}

}

