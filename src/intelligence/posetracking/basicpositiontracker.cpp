#include "basicpositiontracker.h"
#include <common/utils/GPSUtils.h>
#include <common/logger/logger.h>
#include <common/config/configmanager.h>

BasicPositionTracker::BasicPositionTracker(GPS *gps, IMU *imu)
    : _gps(gps),
      _imu(imu)
{
    originPointsRecorded = 0;
    if(_gps != nullptr)
        connect(_gps, SIGNAL(onNewData(GPSData)), this, SLOT(onNewGPS(GPSData)));
    if(_imu != nullptr)
        connect(_imu, SIGNAL(onNewData(IMUData)), this, SLOT(onNewIMU(IMUData)));
}

BasicPositionTracker::~BasicPositionTracker()
{
    if(_gps != nullptr)
        disconnect(_gps, SIGNAL(onNewData(GPSData)), this, SLOT(onNewGPS(GPSData)));
    if(_imu != nullptr)
        disconnect(_imu, SIGNAL(onNewData(IMUData)), this, SLOT(onNewIMU(IMUData)));
}

RobotPosition BasicPositionTracker::GetPosition()
{
    return currentPosition;
}

void BasicPositionTracker::Reset()
{
    origin.Lat(0);
    origin.Long(0);
    origin.Heading(0);
    originPointsRecorded = 0;
}

void BasicPositionTracker::ChangeGPS(GPS *gps)
{

    if(_gps != nullptr)
        disconnect(_gps, SIGNAL(onNewData(GPSData)), this, SLOT(onNewGPS(GPSData)));
    _gps = gps;
    if(_gps != nullptr)
        connect(_gps, SIGNAL(onNewData(GPSData)), this, SLOT(onNewGPS(GPSData)));
}

void BasicPositionTracker::ChangeIMU(IMU *imu)
{
    if(_imu != nullptr)
        disconnect(_imu, SIGNAL(onNewData(IMUData)), this, SLOT(onNewIMU(IMUData)));
    _imu = imu;
    if(_imu != nullptr)
        connect(_imu, SIGNAL(onNewData(IMUData)), this, SLOT(onNewIMU(IMUData)));
}

void BasicPositionTracker::onNewGPS(GPSData data)
{
    int numPointsForOrigin = ConfigManager::Instance().getValue("BasicPoseTracker", "PointsForOrigin", 300);
    if(originPointsRecorded < numPointsForOrigin)
    {
        origin.Lat(  origin.Lat()  + data.Lat() );
        origin.Long( origin.Long() + data.Long() );
        originPointsRecorded++;
        onOriginPercentage((int)( ( (double)originPointsRecorded/(double)numPointsForOrigin) * 100 ));
        if(originPointsRecorded == numPointsForOrigin)
        {
            std::stringstream msg;
            origin.Lat(origin.Lat() / (double)numPointsForOrigin);
            origin.Long(origin.Long() / (double)numPointsForOrigin);
            msg << "Position tracker origin found : LAT " << origin.Lat() << "\tLONG " << origin.Long();
            Logger::Log(LogLevel::Info, msg.str());
        }
        return;
    }
    GPSUtils::coordsToMetricXY(origin.Lat(), origin.Long(), data.Lat(), data.Long(), currentPosition.X, currentPosition.Y);
    onNewPosition(currentPosition);
}

void BasicPositionTracker::onNewIMU(IMUData data)
{
    currentPosition.Heading = data.Yaw;
    onNewPosition(currentPosition);
}
