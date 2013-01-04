#include "sensors/GPS/GPS.hpp"
#include "sensors/GPS/HemisphereA100GPS.h"
#include <iostream>

using namespace IGVC::Sensors;

class MyGPSListener : public GPSListener
{
    void onNewStateAvailable(void* state)
    {
        GPSState* myState;
        myState = (GPSState*)state;

        std::cout << myState->lat << "\t" << myState->lon << std::endl;
    }
};

int main()
{

    HemisphereA100GPS gps;

    std::cout << "GPS device initialized. Press Ctrl+C to quit." << std::endl;

    MyGPSListener listener;
    gps.addListener(&listener);
    while(true);

 /*   while(true)
    {
        if(gps.StateIsAvailable())
        {
            GPSState state = gps.GetState();
            std::cout << state.lat << "\t" << state.lon << std::endl;
        }
    }
*/
}
