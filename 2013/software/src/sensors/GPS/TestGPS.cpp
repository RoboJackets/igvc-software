#include "sensors/GPS/GPS.hpp"
#include "sensors/GPS/HemisphereA100GPS.h"
#include <iostream>

using namespace IGVC::Sensors;

int main()
{
    HemisphereA100GPS gps;
    while(true)
    {
        if(gps.StateIsAvailable())
        {
            GPSState state = gps.GetState();
            std::cout << state.lat << "\t" << state.lon << std::endl;
        }
    }
}
