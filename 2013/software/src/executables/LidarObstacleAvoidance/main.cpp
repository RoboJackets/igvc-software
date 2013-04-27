#include <sensors/lidar/NAV200.h>
#include <actuators/motors/OSMC_driver/OSMC_driver.hpp>
#include <iostream>

using namespace IGVC::Sensors;
using namespace std;

class LOA {
private:
    Lidar* _lidar;
    OSMC_driver* _driver;

public:
    LOA(Lidar *lidar, OSMC_driver* driver)
    : LOnNewLidarFrame(this)
    {
        _lidar = lidar;
        _driver = driver;
        _lidar->onNewData += &LOnNewLidarFrame;
        cout << "Running..." << endl;
    }

    void Stop() {
        _lidar->onNewData -= &LOnNewLidarFrame;
        _driver->stopMotors();
    }

private:
    void OnNewLidarFrame(LidarState data) {
        cout << "data" << endl;
        int* buckets = new int[3];
        for(int i = 0; i < 1024; i++)
        {
            LidarPoint point = data.points[i];
            if(point.valid && point.distance < 1.5/*meters*/)
            {
                double angle = point.angle;
                if(point.angle < 0) {
                    angle += 2.0 * M_PI;
                }
                if(angle > M_PI / 4.0 && angle < 3.0 * M_PI / 4.0)
                    buckets[0]++;
                else if(angle > 3.0 * M_PI / 4.0 && angle < 5.0 * M_PI / 4.0)
                    buckets[1]++;
                else if(angle > 5.0 * M_PI / 4.0 && angle < 7.0 * M_PI / 4.0)
                    buckets[2]++;
                else
                    buckets[3]++;
            }
        }
        int minInd = 0;
        for(int i = 0; i < 4; i++)
            if(buckets[i] < buckets[minInd])
                minInd = i;
        int threshold = 50;
        if(buckets[0] > threshold && buckets[1] > threshold && buckets[2] > threshold && buckets[3] > threshold)
            cout << "No open path." << endl;
        if(buckets)
        switch(minInd){
        case 0:
            cout << "Forward" << endl;
            break;
        case 1:
            cout << "Left" << endl;
            break;
        case 2:
            cout << "Back ***LOOK OUT***" << endl;
            break;
        case 3:
            cout << "Right" << endl;
            break;
        }
    }
    LISTENER(LOA, OnNewLidarFrame, LidarState);
};

int main()
{
    NAV200 lidar;
    OSMC_driver driver;
    LOA loa(&lidar, &driver);
    cout << "Press enter to quit." << endl;
    cin.ignore();
    loa.Stop();
    return 0;
}
