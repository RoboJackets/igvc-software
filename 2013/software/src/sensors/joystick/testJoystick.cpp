#include "Joystick.h"
#include "actuators/motors/OSMC_driver/OSMC_driver.hpp"
#include <iostream>
#include "serial/ASIOSerialPort.h"
using namespace std;

int main(int argc, char* args[] )
{

    Joystick Logi; //Joystick Object
    OSMC_driver driver; //Motor Driver Object
    sleep(1);



    bool ev; //Catch return status of Joystick read function

    //Initialize Joystick subsystem
    if(!Logi.initJoystick())
    {
        return 1;
    }

    driver.stopMotors();
    uint16_t Timer=0;
    while(true)
    {

        //read Joystick data

        if(Logi.readJoystick())
        {
            //Display Joystick data
            //Logitech.displayJoystick();
            Logi.getPWM();

        }

        if(Logi.CycleCheck())
        {
            Logi.smoothPWM();
            driver.setRightLeftPwm(Logi.RPWM,Logi.RDirection,Logi.LPWM,Logi.LDirection);
            cout<<"LPWM:"<<Logi.LPWM<<"    ";
            cout<<"RPWM:"<<Logi.RPWM<<"\n";
            cout<<"jLPWM:"<<Logi.jLPWM<<"    ";
            cout<<"jRPWM:"<<Logi.jRPWM<<"\n";
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




