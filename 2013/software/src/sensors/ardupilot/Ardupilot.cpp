#include "sensors/ardupilot/Ardupilot.hpp"
#include <string.h>
#include <iostream>
#include <sstream>
#include "sensors/DataStructures/IMUData.hpp"

using namespace std;

Ardupilot::Ardupilot():
ardupilotPort("/dev/ttyIMU", 115200)
{

}

Ardupilot::~Ardupilot()
{
    ardupilotPort.close();
}

void Ardupilot::update()
{
    string data = ardupilotPort.readln();
    istringstream iss(data);

    string check;
    iss>>check;

    if(check == "A")
    {
        string headingS;
        iss>>headingS;
        double heading = atof(headingS.c_str());

        string speedS;
        iss>>speedS;
        double speed = atof(speedS.c_str());

        string positionXS;
        iss>>positionXS;
        double positionX = atof(positionXS.c_str());

        string positionYS;
        iss>>positionYS;
        double positionY = atof(positionYS.c_str());

        cout<<heading<<"\t"<<speed<<"\t"<<positionX<<"\t"<<positionY<<endl;
    }
}

void Ardupilot::write(char a)
{
    std::ostringstream s;
    s<<a;
    ardupilotPort.write(s.str());
}
