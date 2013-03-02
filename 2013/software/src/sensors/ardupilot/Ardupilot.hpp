#ifndef ARDUPILOT_H
#define ARDUPILOT_H

#include "serial/ASIOSerialPort.h"

using namespace std;

class Ardupilot
{
    public:
        Ardupilot();
        ~Ardupilot();
        void update();
        void write(char a);

};

#endif // ARDUPILOT_H
