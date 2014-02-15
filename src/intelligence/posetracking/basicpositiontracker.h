#ifndef BASICPOSITIONTRACKER_H
#define BASICPOSITIONTRACKER_H

#include <common/datastructures/GPSData.hpp>
#include <common/datastructures/IMUData.hpp>
#include <common/datastructures/robotposition.hpp>
#include <common/events/Event.hpp>

/**
 * @brief Tracks the robots position relative to it's starting location in meters.
 */
class BasicPositionTracker
{
public:
    BasicPositionTracker();

    RobotPosition GetPosition();

    Event<RobotPosition> onNewPosition;

private:
    void onNewGPS(GPSData data);
    void onNewIMU(IMUData data);

    RobotPosition currentPosition;
    GPSData origin;

    int originPointsRecorded;

public:
    LISTENER(BasicPositionTracker, onNewGPS, GPSData)
    LISTENER(BasicPositionTracker, onNewIMU, IMUData)
};

#endif // BASICPOSITIONTRACKER_H
