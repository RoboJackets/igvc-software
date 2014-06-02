#ifndef BASICPOSITIONTRACKER_H
#define BASICPOSITIONTRACKER_H

#include <hardware/sensors/gps/GPS.hpp>
#include <hardware/sensors/IMU/IMU.h>
#include <common/datastructures/robotposition.hpp>
#include <QObject>

/**
 * @brief Tracks the robots position relative to it's starting location in meters.
 */
class BasicPositionTracker : public QObject
{
    Q_OBJECT
public:
    BasicPositionTracker(std::shared_ptr<GPS> gps, std::shared_ptr<IMU> imu);

    ~BasicPositionTracker();

    RobotPosition GetPosition();

    void Reset();

    void ChangeGPS(std::shared_ptr<GPS> gps);
    void ChangeIMU(std::shared_ptr<IMU> imu);

    RobotPosition WaypointToPosition(GPSData waypoint);

signals:
    void onNewPosition(RobotPosition);
    void onOriginPercentage(int);

private slots:
    void onNewGPS(GPSData data);
    void onNewIMU(IMUData data);

private:
    RobotPosition currentPosition;
    GPSData origin;

    std::shared_ptr<GPS> _gps;
    std::shared_ptr<IMU> _imu;

    int originPointsRecorded;
};

#endif // BASICPOSITIONTRACKER_H
