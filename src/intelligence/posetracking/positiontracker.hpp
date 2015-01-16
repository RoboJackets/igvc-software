#ifndef POSITIONTRACKER_HPP
#define POSITIONTRACKER_HPP

#include <common/module.hpp>
#include <common/datastructures/IMUData.hpp>
#include <common/datastructures/GPSData.hpp>
#include <common/datastructures/robotposition.hpp>

class PositionTracker : public Module {
Q_OBJECT
public:
    PositionTracker() { }

    virtual ~PositionTracker() { }

    virtual void Reset() = 0;

    virtual const RobotPosition &GetPosition() = 0;

    virtual const GPSData &GetOrigin() = 0;

signals:
    void onNewPosition(const RobotPosition &);
    void onOriginPercentage(int);

public slots:
    virtual void onIMUData(IMUData) = 0;
    virtual void onGPSData(const GPSData &) = 0;
};

#endif // POSITIONTRACKER_HPP
