#include "PoseTracker.h"

PoseTracker::PoseTracker(GPS* gps, Ardupilot* imu) :
    LonNewGPSData(this),
    LonNewIMUData(this)
{
    if(gps) gps->onNewData += &LonNewGPSData;
    if(imu) imu->onNewIMUData += &LonNewIMUData;
}

void PoseTracker::onNewGPSData(GPSData data)
{
    if(data.Quality() == GPS_QUALITY_WAAS)
    {
        _latestGPS = data;
        triggerNewPose();
    }
}

void PoseTracker::onNewIMUData(IMUData data)
{
    _latestIMU = data;
    triggerNewPose();
}

void PoseTracker::triggerNewPose()
{
    Pose p;
    p.lat = _latestGPS.Lat();
    p.lon = _latestGPS.Long();
    p.heading = _latestIMU.Heading();
    onNewPose(p);
}

PoseTracker::~PoseTracker()
{
}
