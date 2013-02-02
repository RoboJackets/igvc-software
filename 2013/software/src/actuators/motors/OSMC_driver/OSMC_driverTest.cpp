

#include <string.h>
#include "serial/ASIOSerialPort.h"
#include <iostream>
#include "OSMC_driver.hpp"

int main()
{
    OSMC_driver driver;

	std::cout << "Going Forward" << std::endl;
    driver.setPwm(100,1);
    driver.stopMotors();
}
