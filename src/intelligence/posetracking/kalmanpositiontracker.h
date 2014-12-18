#ifndef KALMANPOSITIONTRACKER_H
#define KALMANPOSITIONTRACKER_H

#include <array>
#include "positiontracker.hpp"
#include <common/datastructures/robotposition.hpp>
#include <hardware/sensors/IMU/IMU.h>
#include <hardware/sensors/gps/GPS.hpp>

class KalmanPositionTracker : public PositionTracker
{
Q_OBJECT

public:
    KalmanPositionTracker(std::shared_ptr<IMU> imu = std::shared_ptr<IMU>(nullptr), std::shared_ptr<GPS> gps = std::shared_ptr<GPS>(nullptr));
    ~KalmanPositionTracker();

    bool isWorking();

    const RobotPosition &GetPosition() {
        return _currentEstimate;
    }

    void Reset();

    const GPSData &GetOrigin() {
        return _origin;
    }

public slots:
    void onIMUData(IMUData);
    void onGPSData(GPSData);

private:
    class TrackerPose {
    public:
        GaussianVariable<double> Latitude;
        GaussianVariable<double> Longitude;
        GaussianVariable<double> Heading;
    };

    TrackerPose _currentInternalEstimate;

    RobotPosition _currentEstimate;

    double _lastGPSTime;
    double _lastIMUTime;

    std::array<GPSData, 5> _GPSDataBuffer;

    GPSData _origin;
    int _numOriginPointsRecorded;

    void predict();

    void emitCurrentEstimate();
};

#endif // KALMANPOSITIONTRACKER_H
