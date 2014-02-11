#include "basicpositiontracker.h"
#include <common/utils/GPSUtils.h>
#include <common/logger/logger.h>

BasicPositionTracker::BasicPositionTracker()
    : NUMBER_OF_POINTS_FOR_ORIGIN(300),
      LonNewGPS(this),
      LonNewIMU(this)
{
}

RobotPosition BasicPositionTracker::GetPosition()
{
    return currentPosition;
}

void BasicPositionTracker::onNewGPS(GPSData data)
{
    if(originPointsRecorded < NUMBER_OF_POINTS_FOR_ORIGIN)
    {
        origin.Lat(  origin.Lat()  + ( data.Lat()  / NUMBER_OF_POINTS_FOR_ORIGIN ) );
        origin.Long( origin.Long() + ( data.Long() / NUMBER_OF_POINTS_FOR_ORIGIN ) );
        originPointsRecorded++;
        if(originPointsRecorded == NUMBER_OF_POINTS_FOR_ORIGIN)
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
