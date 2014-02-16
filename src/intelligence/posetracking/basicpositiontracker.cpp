#include "basicpositiontracker.h"
#include <common/utils/GPSUtils.h>
#include <common/logger/logger.h>
#include <common/config/configmanager.h>

BasicPositionTracker::BasicPositionTracker()
    : LonNewGPS(this),
      LonNewIMU(this)
{
    originPointsRecorded = 0;
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
        origin.Lat(  origin.Lat()  + data.Lat() );
        origin.Long( origin.Long() + data.Long() );
        originPointsRecorded++;
        std::cout << numPointsForOrigin - originPointsRecorded << " " << data.Lat() << " " << data.Long() << std::endl;
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
