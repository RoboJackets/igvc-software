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
    BasicPositionTracker(GPS *gps, IMU *imu);

    ~BasicPositionTracker();

    RobotPosition GetPosition();

    void Reset();

    void ChangeGPS(GPS *gps);
    void ChangeIMU(IMU *imu);

signals:
    void onNewPosition(RobotPosition);
    void onOriginPercentage(int);

private slots:
    void onNewGPS(GPSData data);
    void onNewIMU(IMUData data);

private:
    RobotPosition currentPosition;
    GPSData origin;

    GPS *_gps;
    IMU *_imu;

    int originPointsRecorded;
};

#endif // BASICPOSITIONTRACKER_H
