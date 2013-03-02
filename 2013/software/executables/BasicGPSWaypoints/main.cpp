
//#include "sensors/RobotPosition.h"
//#include "actuators/motors/OSMC_driver/OSMC_driver.hpp"
#include <cmath>
#include <iostream>

//using namespace IGVC::Sensors;

float headingBetweenPoints(float latA, float lonA, float latB, float lonB)
{
    return fmod(
                    atan2(
                      sin(lonA-lonB)*cos(latB) ,
                      cos(latA)*sin(latB)-sin(latA)*cos(latB)*cos(lonA-lonB) ),
                    2.0 * M_PI)
                * (180.0 / M_PI);
}

int main()
{

    std::cout << headingBetweenPoints(33.787360, -84.406981, 33.787380, -84.407);
//    RobotPosition position();
//    OSMC_driver driver();
}
