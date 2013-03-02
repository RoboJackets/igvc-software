#include <string.h>
#include "serial/ASIOSerialPort.h"
#include <iostream>
#include "sensors/ardupilot/Ardupilot.hpp"

int main()
{
    Ardupilot ardupilot;
    std::cout<<"Running"<<endl;
    sleep(1);
    while(true)
    {
//        ardupilot.write('A');
        ardupilot.update();
        usleep(100000);
    }
}
