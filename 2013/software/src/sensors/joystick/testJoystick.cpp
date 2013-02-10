#include "Joystick.h"
#include "actuators/motors/OSMC_driver/OSMC_driver.hpp"
#include <iostream>
#include "serial/ASIOSerialPort.h"
using namespace std;

int main(int argc, char* args[] )
{

    Joystick Logitech; //Joystick Object
    OSMC_driver driver; //Motor Driver Object
    int Timer;
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


        if(ev)
        {
            //Display Joystick data
            //Logitech.displayJoystick();

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

//            std::cout<<"leftPWM =  "<<leftPWM<<"\n";
//            std::cout<<"rightPWM =  "<<rightPWM<<"leftDirection<<"\n";
//            std::cout<<"rightdirection =  "<<rightDirection\n";
//            std::cout<<"leftdirection =  "<<<<"\n";


        }


        setleftPWM=leftPWM;
        setrightPWM=rightPWM;
        cout<<"setleftPWM:"<<setleftPWM<<"    ";
        cout<<"setrightPWM:"<<setrightPWM<<"\n";
        usleep(100);
        Timer++;
        if (Timer==2000)
        {
            driver.setRightLeftPwm((char)setrightPWM,0,(char)setleftPWM,0);
            Timer=0;
        }





//                  if(leftPWMsetleftPWM)
//                {
//                    if(setleftPWM<255)
//                    {
//                        setleftPWM+=10;
//                        if (setleftPWM>=255)
//                        {
//                            setleftPWM=255;
//                        }
//                        cout<<"setleftPWM:"<<setleftPWM<<"\n";
//                        driver.setRightLeftPwm((char)setleftPWM,0,(char)setleftPWM,0);
//                        usleep(200000);
//                    }
//                }
//
//            else if(leftPWM<setleftPWM)
//                {
//                    if(setleftPWM>0)
//                    {
//                        setleftPWM-=10;
//                        if (setleftPWM<=0)
//                        {
//                            setleftPWM=0;
//                        }
//                        cout<<"setleftPWM:"<<setleftPWM<<"\n";
//                        driver.setRightLeftPwm((char)setleftPWM,0,(char)setleftPWM,0);
//                        usleep(200000);
//                    }
//                }

    }

    return 0;

}




