#ifndef BASICPOSITIONTRACKER_H
#define BASICPOSITIONTRACKER_H

#include <hardware/sensors/gps/GPS.hpp>
#include <hardware/sensors/IMU/IMU.h>
#include "positiontracker.hpp"
#include <memory>

/**
 * @brief Tracks the robots position relative to it's starting location in meters.
 */
class BasicPositionTracker : public PositionTracker
{
    Q_OBJECT
public:
    BasicPositionTracker(std::shared_ptr<GPS> gps = std::shared_ptr<GPS>(nullptr), std::shared_ptr<IMU> imu = std::shared_ptr<IMU>(nullptr));

    ~BasicPositionTracker();

    const RobotPosition &GetPosition();

    void Reset();

    RobotPosition WaypointToPosition(GPSData waypoint);

    bool isWorking();

    const GPSData &GetOrigin() {
        return origin;
    }

signals:
    void onNewPosition(RobotPosition);
    void onOriginPercentage(int);

private slots:
    void onGPSData(const GPSData &data);
    void onIMUData(IMUData data);

private:
    RobotPosition currentPosition;
    GPSData origin;

    int originPointsRecorded;
};

#endif // BASICPOSITIONTRACKER_H
