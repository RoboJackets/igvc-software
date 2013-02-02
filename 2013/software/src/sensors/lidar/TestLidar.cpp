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
//        file.open("data.txt");
    }

    void onNewStateAvailable(LidarState state)
    {

        std::cout << " Test Output " << std::endl; //std::setprecision(9)  << state.lat << "\t" << state.lon
//        file << std::setprecision(9)  << state.lat << "\t" << state.>lon << std::endl;
    }

    LISTENER(MyLidarListener, onNewStateAvailable, LidarState);

    ~MyLidarListener()
    {
//        file.close();
    }
};

int main()
{
    cout << "Active!" << endl;
    NAV200 lidar;
    cout << "Process ended." << endl;
	return 0;
}
