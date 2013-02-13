#include "sensors/lidar/Lidar.h"
#include "sensors/lidar/NAV200.h"
#include <iostream>
#include <fstream>
#include <iomanip>


using namespace IGVC::Sensors;
using namespace std;

class MyLidarListener
{
private:
    ofstream file;
public:
    MyLidarListener(Lidar* lidar)
        : LonNewStateAvailable(this)
    {
        lidar->onNewData += &LonNewStateAvailable;
    }

    void onNewStateAvailable(LidarState state)
    {
        cout << state.points[0].distance << endl;
    }

    LISTENER(MyLidarListener, onNewStateAvailable, LidarState);

    ~MyLidarListener()
    {
    }
};



int main()
{
    std::cout << "Lidar initialized. Press Ctrl+C to quit." << std::endl;

    NAV200 lidar;

    // Instantiate a lidar listener
    MyLidarListener listener(&lidar);

    // Keep the main app running while the Lidar thread runs
    while(true)
    {
        cout << "test" << endl;
        char in;
        cin >> in;
        if(in == 27)
            break;
    }

    lidar.stop();

    cout << "Process ended." << endl;

	return 0;
}
