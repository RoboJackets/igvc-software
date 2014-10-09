#include "basicpositiontracker.h"
#include <common/utils/GPSUtils.h>
#include <common/logger/logger.h>
#include <common/config/configmanager.h>

BasicPositionTracker::BasicPositionTracker(std::shared_ptr<GPS> gps, std::shared_ptr<IMU> imu)
    : currentPosition(0,0,0),
      _gps(gps),
      _imu(imu)
{
    qRegisterMetaType<RobotPosition>("RobotPosition");
    originPointsRecorded = 0;
    if(_gps.get() != nullptr)
        connect(_gps.get(), SIGNAL(onNewData(GPSData)), this, SLOT(onNewGPS(GPSData)));
    if(_imu.get() != nullptr)
        connect(_imu.get(), SIGNAL(onNewData(IMUData)), this, SLOT(onNewIMU(IMUData)));
}

BasicPositionTracker::~BasicPositionTracker()
{
    if(_gps.get() != nullptr)
        disconnect(_gps.get(), SIGNAL(onNewData(GPSData)), this, SLOT(onNewGPS(GPSData)));
    if(_imu.get() != nullptr)
        disconnect(_imu.get(), SIGNAL(onNewData(IMUData)), this, SLOT(onNewIMU(IMUData)));
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

void BasicPositionTracker::ChangeGPS(std::shared_ptr<GPS> gps)
{

    if(_gps.get() != nullptr)
        disconnect(_gps.get(), SIGNAL(onNewData(GPSData)), this, SLOT(onNewGPS(GPSData)));
    _gps = gps;
    if(_gps.get() != nullptr)
        connect(_gps.get(), SIGNAL(onNewData(GPSData)), this, SLOT(onNewGPS(GPSData)));
}

void BasicPositionTracker::ChangeIMU(std::shared_ptr<IMU> imu)
{
    if(_imu.get() != nullptr)
        disconnect(_imu.get(), SIGNAL(onNewData(IMUData)), this, SLOT(onNewIMU(IMUData)));
    _imu = imu;
    if(_imu.get() != nullptr)
        connect(_imu.get(), SIGNAL(onNewData(IMUData)), this, SLOT(onNewIMU(IMUData)));
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
            origin.Lat(origin.Lat() / (double)numPointsForOrigin);
            origin.Long(origin.Long() / (double)numPointsForOrigin);
            Logger::Log(LogLevel::Info, "[BasicPositionTracker] Origin found: LAT " + std::to_string(origin.Lat()) + "\tLONG " + std::to_string(origin.Long()));
        }
        return;
    }
    GPSUtils::coordsToMetricXY(origin.Lat(), origin.Long(), data.Lat(), data.Long(), currentPosition.X, currentPosition.Y);
    onNewPosition(currentPosition);
}

void BasicPositionTracker::onNewIMU(IMUData data)
{
    currentPosition.Heading = ( (data.Yaw < 0) ? data.Yaw + 360 : data.Yaw );

    onNewPosition(currentPosition);
}

RobotPosition BasicPositionTracker::WaypointToPosition(GPSData waypoint)
{
    RobotPosition ret;
    GPSUtils::coordsToMetricXY(origin.Lat(), origin.Long(), waypoint.Lat(), waypoint.Long(), ret.X, ret.Y);
    return ret;
}
