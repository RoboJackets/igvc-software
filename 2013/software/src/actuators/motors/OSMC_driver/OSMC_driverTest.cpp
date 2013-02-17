#include <string.h>
#include "serial/ASIOSerialPort.h"
#include <iostream>
#include "OSMC_driver.hpp"

int main()
{
    OSMC_driver driver;


	std::cout << "Going Forward" << std::endl;
	/*
	//driver.setPwm(255,1);

	sleep(1);

//	driver.arduinoCheck();

    for (int i=0;i<255;i+=1)

    {
        driver.setPwm(i,1);
        usleep(250000);
    }
    sleep(1);
    cout<<"Full Speed"<<endl;

    sleep(2);

    cout<<"Two seconds have passed"<<endl;
    */

    driver.turn(1.0,75,driver.RIGHT);
   //driver.setRightLeftPwm(75,0,75,1);
    sleep(10);

    driver.stopMotors();

}
