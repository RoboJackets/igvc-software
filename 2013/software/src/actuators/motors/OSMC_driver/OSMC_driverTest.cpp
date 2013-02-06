

#include <string.h>
#include "serial/ASIOSerialPort.h"
#include <iostream>
#include "OSMC_driver.hpp"

int main()
{
    OSMC_driver driver;
	std::cout << "Going Forward" << std::endl;
	//driver.setPwm(255,1);

	sleep(1);

	driver.arduinoCheck();

    for (int i=0;i<255;i+=5)
    {
        driver.setPwm(i,1);
        usleep(250000);
    }
    cout<<"Full Speed"<<endl;

    sleep(10);

    driver.stopMotors();
}
