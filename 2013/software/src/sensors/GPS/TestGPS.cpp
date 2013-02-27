#include "sensors/GPS/GPS.hpp"
#include "sensors/GPS/HemisphereA100GPS.h"
#include <iostream>
#include <fstream>
#include <iomanip>

using namespace IGVC::Sensors;
using namespace std;

class MyGPSListener
{
private:
    ofstream file;
public:
    MyGPSListener(GPS* gps)
        : LonNewStateAvailable(this)
    {
        gps->onNewData += &LonNewStateAvailable;
//        file.open("data.txt");
    }

    void onNewStateAvailable(GPSData state)
    {

        std::cout << std::setprecision(9)  << state.Lat() << "\t" << state.Long() << "\t" << state.Heading() << std::endl;
//        file << std::setprecision(9)  << state.lat << "\t" << state.>lon << std::endl;
    }

    LISTENER(MyGPSListener, onNewStateAvailable, GPSData);

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
    MyGPSListener listener(&gps);

    // Keep the main app running while the GPS thread runs
    while(true);
}
