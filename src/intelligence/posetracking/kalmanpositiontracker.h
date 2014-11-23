#ifndef KALMANPOSITIONTRACKER_H
#define KALMANPOSITIONTRACKER_H

#include <common/module.hpp>
#include <hardware/sensors/IMU/IMU.h>
#include <hardware/sensors/gps/GPS.hpp>

class KalmanPositionTracker : public Module
{
public:
    KalmanPositionTracker();

    bool isWorking();

public slots:
    void OnIMUData(IMUData);

};

#endif // KALMANPOSITIONTRACKER_H
