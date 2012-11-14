//============================================================================
// Name        : SerialPortTest.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <string.h>

#include "ASIOSerialPort.h"
#include <iostream>

using namespace std;

int main() {
	std::cout << "Testing serial API..." << std::endl;
	ASIOSerialPort serialPort("/dev/arduino", 9600);
	for(int i=0; i < 6; i++) {
		serialPort.write("T");
		sleep(1);
	}
	while(true) {
		std::cout << serialPort.readln() << std::endl;
	}
	serialPort.close();
}
