#ifndef POSETRACKER_H
#define POSETRACKER_H

#include <sensors/GPS/GPS.hpp>
#include <sensors/ardupilot/Ardupilot.hpp>
#include <events/Event.hpp>

using namespace IGVC::Sensors;

struct Pose {
    double lat;
    double lon;
    double heading;
};

class PoseTracker
{
    public:
        PoseTracker(GPS* gps, Ardupilot* imu);
        virtual ~PoseTracker();

        Event<Pose> onNewPose;
    protected:
        void onNewGPSData(GPSData data);
        LISTENER(PoseTracker, onNewGPSData, GPSData);
        void onNewIMUData(IMUData data);
        LISTENER(PoseTracker, onNewIMUData, IMUData);
        void triggerNewPose();
    private:
        GPSData _latestGPS;
        IMUData _latestIMU;
};

#endif // POSETRACKER_H
