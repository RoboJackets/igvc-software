#include "Joystick.h"
#include "actuators/motors/OSMC_driver/OSMC_driver.hpp"
#include <iostream>
#include "serial/ASIOSerialPort.h"
using namespace std;

int main(int argc, char* args[] )
{

    Joystick Logitech; //Joystick Object
    OSMC_driver driver; //Motor Driver Object
    sleep(5);

    volatile int leftPWM;
    volatile int rightPWM;
    int leftDirection, rightDirection;

    bool ev; //Catch return status of Joystick read function

    //Initialize Joystick subsystem
    if(!Logitech.initJoystick())
    {
        return 1;
    }

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

            rightPWM=abs(rightPWM);

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
         //   driver.setRightLeftPwm((char)rightPWM,1,(char)leftPWM,1);
            driver.setRightLeftPwm(rightPWM, 1, leftPWM, 1);
            usleep(500000);
            driver.setPwm(leftPWM,1);
        }

    }

    return 0;

}




