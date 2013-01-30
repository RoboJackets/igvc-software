

#include <string.h>
#include "serial/ASIOSerialPort.h"
#include <iostream>
#include "OSMC_driver.hpp"

int main() {
	std::cout << "Going Forward" << std::endl;
	OSMC_driver driver;

	driver.goForwardOld(0.2,100,1);
//    driver.stopMotors();
}
