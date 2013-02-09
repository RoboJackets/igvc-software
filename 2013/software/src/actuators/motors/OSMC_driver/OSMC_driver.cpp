#include <string.h>

#include "serial/ASIOSerialPort.h"
#include <iostream>
#include "actuators/motors/OSMC_driver/OSMC_driver.hpp"
#include <time.h>

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
    setRightLeftPwm(pwm, dir, pwm, dir);
}

void OSMC_driver::setRightLeftPwm(char pwmRight, char dirRight, char pwmLeft, char dirLeft)
{
    pwmRight = adjustSpeedRight(pwmRight, dirRight);
    dirLeft = adjustDirLeft(dirLeft);
    pwmLeft = adjustSpeedLeft(pwmLeft, dirLeft);
    string w = "SW";
    w.append(intToString(dirRight));
    w.append(" ");
    w.append(intToString(pwmRight));
    w.append(" ");
    w.append(intToString(dirLeft));
    w.append(" ");
    w.append(intToString(pwmLeft));
    cout<<w.c_str()<<endl;
    arduinoRight.write(w);
    string w1 = "SW";
    w1.append(intToString(dirLeft));
    w1.append(" ");
    w1.append(intToString(pwmLeft));
    w1.append(" ");
    w1.append(intToString(dirRight));
    w1.append(" ");
    w1.append(intToString(pwmRight));
    arduinoLeft.write(w1);
}

char OSMC_driver::adjustSpeedRight(char pwm, char dir)
{
    if (dir == 1)
    {
        pwm = 255-pwm;
    }
    if (dir == 0)
    {
        pwm = pwm;
    }
    return (pwm);
}

char OSMC_driver::adjustSpeedLeft(char pwm, char dir)
{
    if (dir == 1)
    {
        pwm = 255-pwm;
    }
    if (dir == 0)
    {
        pwm = pwm;
    }
    return (pwm);
}

char OSMC_driver::adjustDirLeft(char dirLeft)
{
    dirLeft = 1-dirLeft;
    return (dirLeft);
}


void OSMC_driver::turn(double radius, int pwm, int dir)
{
    double v1 = 0;
    double v2 = 0;
    double halfWB = WHEELBASE/2;
    double comp1 = (radius+halfWB)/(radius-halfWB);
    v2 = (2*pwm*comp1)/(1+comp1);
    v1 = 2*pwm-v2;
    if (dir == 1)
    {
        setRightLeftPwm(v1, 0, v2, 0);
    }
    else if(dir == 0)
    {
        setRightLeftPwm(v2, 0, v1, 0);
    }
    else
    {
        cout<<"Bad direction"<<endl;
    }

}

string OSMC_driver::intToString(int input)
{
    std::ostringstream s;
    s << input;
//    cout<< s.str() << endl;
    return s.str();

}
// For use with new robot
/*
void OSMC_driver::goForward(double dist, char pwm, char dir)
{
    string d = "D";
    wd.append(1,dist);
    wd.append(1,dir);
    wd.append(1,pwm);

    arduino.write(wd);

    while(arduino.readln() == "!")
    {
        break;
    }
}

*/

double OSMC_driver::readEncoder()
{
    string r = "R";
    arduinoEncoder.write("R");
    sleep(0.1);
    std::string dist1 = arduinoEncoder.readln();
    double dist = ::atof(dist1.c_str());
    return dist;
}

void OSMC_driver::encoderLoop(double totalDist)
{
    double dist = readEncoder();
    while (dist<totalDist)
    {
        dist = readEncoder();
        cout<<dist<<endl;
        sleep(0.05);
    }
    cout<<"Dropped Loop"<<endl;
    stopMotors();
}

void OSMC_driver::goForwardOld(double totalDist, char pwm, char dir)
{
    string f = "F";
    arduinoEncoder.write(f);
    sleep(1);
    string var;
    var = "";
   var = arduinoEncoder.readln();
 //   cout<<var<<endl;
 //   sleep(1);
    setRightLeftPwm(pwm, dir, pwm, dir);
    encoderLoop(totalDist);
}

void OSMC_driver::stopMotors()		//Stops the motors
{
    setRightLeftPwm(0,1,0,1);
}




