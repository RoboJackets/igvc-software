#include "Joystick.h"
#include "actuators/motors/OSMC_driver/OSMC_driver.hpp"
#include <iostream>
#include "serial/ASIOSerialPort.h"
using namespace std;

int main( int argc, char* args[] )
{
    OSMC_driver driver;
    //Quit flag
    bool quit = false;

    //Make the joystick
    Joystick XBox;
    //OSMC_driver driver;
    //sleep(5);
    //Make the Screen
    Graph myDisplay;
    volatile int leftPWM;
    volatile int rightPWM;
    int leftdirection, rightdirection;
    //The frame rate regulator
    Timer fps;

    //Initialize
    if(!XBox.initJoystick())
    return 1;

    if(!myDisplay.initGraph())
    {
        std::cout<<"Error Initializing Graph"<<"\n";
        return 1;
    }

    fps.initTimer();

     while (quit == false)
     //while (XBox.Quit() ==false && myDisplay.Quit() ==false)
     {

//        fps.start();


      //  while(1)
       // {


            XBox.readJoystick();

            cout << XBox.leftAnalogY << "\t" << XBox.rightAnalogY << endl;

            //set pwm
//            leftPWM=(int)(XBox.leftAnalogY/32768.0*255);
//            if(signbit(leftPWM))
//            leftdirection=0;
//            else
//            leftdirection=1;
//            rightPWM=abs(rightPWM);
//
//            rightPWM=(int)(XBox.rightAnalogY/32768.0*255);
//            if(signbit(rightPWM))
//            rightdirection=0;
//            else
//            rightdirection=1;
//            rightPWM=abs(rightPWM);
//
//
//            std::cout<<"leftPWM =  "<<leftPWM<<"\n";
//            std::cout<<"rightPWM =  "<<rightPWM<<"\n";
//            std::cout<<"leftdirection =  "<<leftdirection<<"\n";
//            std::cout<<"rightdirection =  "<<rightdirection<<"\n";
//
//
//         //   driver.setRightLeftPwm((char)rightPWM,1,(char)leftPWM,1);
//            driver.setRightLeftPwm(rightPWM, 1, leftPWM, 1);
//            usleep(500000);
            //driver.setPwm(leftPWM,1);
            //sleep(3);
         //   XBox.setMotion();
       // }


       // myDisplay.displayGraph();
        //Cap the frame rate
  /*      if( fps.get_ticks() < 5000 / myDisplay.FRAMES_PER_SECOND )
        {
            SDL_Delay ( ( 5000 / myDisplay.FRAMES_PER_SECOND ) - fps.get_ticks() );
        }
*/
     }

    //Clean up
  //  myDisplay.cleanGraph();
    XBox.cleanJoystick();
    std::cout<<"Quitting"<<SDL_GetError()<<"\n";
    return 0;
}
