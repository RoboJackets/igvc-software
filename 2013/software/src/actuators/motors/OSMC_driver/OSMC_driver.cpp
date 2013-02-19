#include <string.h>

#include "serial/ASIOSerialPort.h"
#include <iostream>
#include "actuators/motors/OSMC_driver/OSMC_driver.hpp"
#include <time.h>

using namespace std;

ASIOSerialPort arduinoLeft("/dev/igvc_2012_left_motor_shield", 9600);
ASIOSerialPort arduinoRight("/dev/igvc_2012_right_motor_shield", 9600);
ASIOSerialPort arduinoEncoder("/dev/igvc_2012_right_encoder_shield", 9600);

OSMC_driver::~OSMC_driver()
{
	stopMotors();
	arduinoRight.close();
	arduinoLeft.close();
	arduinoEncoder.close();
}

//Checks we are connected and able to send and receive data from the arduinos
bool OSMC_driver::arduinoCheck()
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

//writes pwm to arduinos setting both motors to the same pwm and direction (0 = forwards, 1 = backwards)
void OSMC_driver::setPwm(char pwm, char dir)
{
    /*
    if(dir==FORWARD)
    {
        setRightLeftPwm(pwm, 0, pwm, 0);
    }
    else if(dir==BACKWARD)
    {

    }
    else
    {
        cout<<"Bad Direction"<<endl;
    }
    */
    setRightLeftPwm(pwm,dir,pwm,dir);
}

// Writes to arduinos a string of ints dirRight, pwmRight, dirLeft, pwmLeft.
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

//adjusts the pwm of the right motors based on the direction
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

//adjusts the pwm of the left motors based on the direction
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

//adjusts the direction of the left motors because they are mirrors of the right motors
char OSMC_driver::adjustDirLeft(char dirLeft)
{
    dirLeft = 1-dirLeft;
    return (dirLeft);
}

//turn the robot at a pwm around a circle of a certain radius with a direction
void OSMC_driver::turn(double radius, int pwm, Direction dir)    //dir = 0 Right  dir = 1 Left
{
    double v1 = 0;
    double v2 = 0;
    double halfWB = WHEELBASE/2;
    double comp1 = (radius+halfWB)/(radius-halfWB);
    v2 = (2*pwm*comp1)/(1+comp1);
    v1 = 2*pwm-v2;
    if (dir == RIGHT)
    {
        setRightLeftPwm(v1, 0, v2, 0);
    }
    else if(dir == LEFT)
    {
        setRightLeftPwm(v2, 0, v1, 0);
    }
    else
    {
        cout<<"Bad direction"<<endl;
    }

}

//Used in writing integers to the arduinos as strings
string OSMC_driver::intToString(int input)
{
    std::ostringstream s;
    s << input;
    return s.str();
}

//Used in writing doubles to the arduinos as strings
string OSMC_driver::doubleToString(double input)
{
    std::ostringstream s;
    s << input;
    return s.str();
}

/*
//Go forward a set distance at a set speed in a certain direction
void OSMC_driver::forward(char pwm, char dir, double dist)
{
    setPwm(pwm, dir)
    string d = " ";
    d.append(doubleToString(dist));
    arduino.write(d);
    //TODO event listener for a ! from arduino
    //while(arduino.readln() != "!") {};
}

void OSMC_driver::turn(double radius, double degree, int pwm, char dir)    //dir = 0 Right  dir = 1 Left
{
    double v1 = 0;
    double v2 = 0;
    double halfWB = WHEELBASE/2;
    double comp1 = (radius+halfWB)/(radius-halfWB);
    v2 = (2*pwm*comp1)/(1+comp1);
    v1 = 2*pwm-v2;
    if (dir == 0)
    {
        setRightLeftPwm(v1, 0, v2, 0);
    }
    else if(dir == 1)
    {
        setRightLeftPwm(v2, 0, v1, 0);
    }
    else
    {
        cout<<"Bad direction"<<endl;
    }
    string r = "SR";
    r.append(doubleToString(degree))
    arduino.write(r);
}
*/

//Dont quite work at the moment
//functions for reading encoder data from the arduinos and moving a set distance
/*
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
    setRightLeftPwm(pwm, dir, pwm, dir);
    encoderLoop(totalDist);
}
*/

//Stops the motors
void OSMC_driver::stopMotors()
{
    setRightLeftPwm(0,1,0,1);
}




