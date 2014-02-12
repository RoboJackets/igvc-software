#include "basicpositiontracker.h"
#include <common/utils/GPSUtils.h>
#include <common/logger/logger.h>
#include <common/config/configmanager.h>

BasicPositionTracker::BasicPositionTracker()
    : LonNewGPS(this),
      LonNewIMU(this)
{
}

RobotPosition BasicPositionTracker::GetPosition()
{
    return currentPosition;
}

void BasicPositionTracker::onNewGPS(GPSData data)
{
    int numPointsForOrigin = ConfigManager::Instance().getValue("BasicPoseTracker", "PointsForOrigin", 300);
    if(originPointsRecorded < numPointsForOrigin)
    {
        origin.Lat(  origin.Lat()  + ( data.Lat()  / numPointsForOrigin ) );
        origin.Long( origin.Long() + ( data.Long() / numPointsForOrigin ) );
        originPointsRecorded++;
        if(originPointsRecorded == numPointsForOrigin)
        {
            std::stringstream msg;
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
