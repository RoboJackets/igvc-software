#include "sensors/GPS/GPS.hpp"
#include "sensors/GPS/HemisphereA100GPS.h"
#include <iostream>
#include <fstream>
#include <iomanip>

using namespace IGVC::Sensors;
using namespace std;

class MyGPSListener : public GPSListener
{
private:
    ofstream file;
public:
    MyGPSListener()
    {
//        file.open("data.txt");
    }
    void onNewStateAvailable(void* state)
    {
        GPSState* myState;
        myState = (GPSState*)state;

        std::cout << std::setprecision(9)  <<  myState->lat << "\t" << myState->lon << std::endl;
//        file << std::setprecision(9)  <<  myState->lat << "\t" << myState->lon << std::endl;
    }
    ~MyGPSListener()
    {
//        file.close();
    }
};

int main()
{

    // Initalize specific gps class
    HemisphereA100GPS gps;

    std::cout << "GPS device initialized. Press Ctrl+C to quit." << std::endl;

    // Instantiate a gps listener
    MyGPSListener listener;

    // Add the new listener to our GPS
    gps.addListener(&listener);

    // Keep the main app running while the GPS thread runs
    while(true);
}
