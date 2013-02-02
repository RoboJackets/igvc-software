

#include <string.h>
#include "serial/ASIOSerialPort.h"
#include <iostream>
#include "OSMC_driver.hpp"

int main() {
	std::cout << "Going Forward" << std::endl;
	OSMC_driver driver;

    for (int i=0; i<200; i++)
    {
        driver.setPwm(i,1);
    }
    sleep(2);
    driver.stopMotors();

}
