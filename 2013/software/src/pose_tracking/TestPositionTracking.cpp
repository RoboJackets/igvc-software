#include <iostream>
#include <sensors/GPS/HemisphereA100GPS.h>
#include <sensors/ardupilot/Ardupilot.hpp>
#include <pose_tracking/PoseTracker.h>

//GPSAccuracy GPSData::NAV200Default = GPSAccuracy(.0001, .0001, 3, 0.01);
GPSAccuracy IMUData::ArduPilotDefault= GPSAccuracy(.001, .001, 1, .1);

using namespace IGVC::Sensors;
using namespace std;

//class PositionTracker
//{
//private:
//    GPSData _latestGPS;
//    IMUData _latestIMU;
//
//public:
//    PositionTracker(GPS* gps, Ardupilot* imu) :
//        LonNewStateAvailable(this),
//        LonNewIMUData(this)
//    {
//        gps->onNewData += &LonNewStateAvailable;
//        imu->onNewIMUData += &LonNewIMUData;
//    }
//
//    void onNewStateAvailable(GPSData state) {
//        _latestGPS = state;
//        _latestGPS.Heading(_latestIMU.Heading());
//        cout << setprecision(15) << _latestGPS.Lat() << "\t" << _latestGPS.Long() << "\t" << _latestGPS.Heading() << endl;
//    }
//    LISTENER(PositionTracker, onNewStateAvailable, GPSData);
//
//    void onNewIMUData(IMUData data) {
//        _latestIMU = data;
//    }
//    LISTENER(PositionTracker, onNewIMUData, IMUData);
//
//    ~PositionTracker()
//    {
//    }
//};

class PoseListener {
public:
    PoseListener(PoseTracker* pt) :
        LonNewPose(this)
    {
        pt->onNewPose += &LonNewPose;
    }

private:
    void onNewPose(Pose pose) {
        cout << pose.lat << "\t" << pose.lon << "\t" << pose.heading << endl;
    }
    LISTENER(PoseListener, onNewPose, Pose);
};

int main()
{
    cout << "Connecting to gps..." << endl;
    HemisphereA100GPS gps;
    cout << "done!" << endl;
    cout << "Connecting to IMU..." << endl;
    Ardupilot IMU;
    cout << "done!" << endl;
    cout << "Instantiating tracker..." << endl;
    PoseTracker listener(&gps, &IMU);
    cout << "done!" << endl;
    cout << "Instantiating listener..." << endl;
    PoseListener pl(&listener);
    cout << "done!" << endl;
    cout << "Entering execution loop." << endl;

    while(true);
}
