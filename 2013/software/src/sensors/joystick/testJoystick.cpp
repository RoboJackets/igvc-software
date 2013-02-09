#include "Joystick.h"
#include "actuators/motors/OSMC_driver/OSMC_driver.hpp"
#include <iostream>
#include "serial/ASIOSerialPort.h"
using namespace std;

int main(int argc, char* args[] )
{

    Joystick Logitech; //Joystick Object
    OSMC_driver driver; //Motor Driver Object
    sleep(1);

    volatile int leftPWM, setleftPWM=0;
    volatile int rightPWM, setrightPWM=0;
    int leftDirection, rightDirection;

    bool ev; //Catch return status of Joystick read function

    //Initialize Joystick subsystem
    if(!Logitech.initJoystick())
    {
        return 1;
    }

    driver.stopMotors();

    while(true)
    {

        //read Joystick data
        ev=Logitech.readJoystick();

        //Display Joystick data
        if(ev)
        {
            Logitech.displayJoystick();

            //Set Motor PWM

            leftPWM=(int)(Logitech.leftYAxis/32768.0*255);
            if(signbit(leftPWM))
            {
                leftDirection=0;
            }

            else
            {
                leftDirection=1;
            }
            leftPWM=abs(leftPWM);

            rightPWM=(int)(Logitech.rightYAxis/32768.0*255);
            if(signbit(rightPWM))
            {
                rightDirection=0;
            }

            else
            {
                rightDirection=1;
            }
            rightPWM=abs(rightPWM);

            std::cout<<"leftPWM =  "<<leftPWM<<"\n";
            std::cout<<"rightPWM =  "<<rightPWM<<"\n";
            std::cout<<"leftdirection =  "<<leftDirection<<"\n";
            std::cout<<"rightdirection =  "<<rightDirection<<"\n";

            if(leftPWM>setleftPWM)
                {
                    if(setleftPWM<255)
                    {
                        setleftPWM+=1;
                        std:cout<<"setleftPWM:"<<setleftPWM<<"\n";
                        driver.setRightLeftPwm((char)setleftPWM,0,(char)setleftPWM,0);
                        usleep(200000);
                    }
                }

            else if(leftPWM<setleftPWM)
                {
                    if(setleftPWM>0)
                    {
                        setleftPWM-=1;
                        std:cout<<"setleftPWM:"<<setleftPWM<<"\n";
                        driver.setRightLeftPwm((char)setleftPWM,0,(char)setleftPWM,0);
                        usleep(200000);
                    }
                }
        }

    }

    return 0;

}




