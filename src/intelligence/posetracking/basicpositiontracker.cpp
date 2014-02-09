#include "basicpositiontracker.h"

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
        return;
    }
    // GPS -> Cartesian deltas (origin, data, &currentPosition.X, &currentPosition.Y);
    onNewPosition(currentPosition);
}

void BasicPositionTracker::onNewIMU(IMUData data)
{
    currentPosition.Heading = data.Yaw;
    onNewPosition(currentPosition);
}
