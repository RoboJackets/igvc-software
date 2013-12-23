#ifndef POSITIONTRACKER_H
#define POSITIONTRACKER_H

#include "gaussianvariable.hpp"
#include <iostream>
#include <hardware/sensors/gps/GPS.hpp>
#include <hardware/actuators/motors/MotorDriver.hpp>
#include <hardware/sensors/IMU/IMU.h>

class Position
{
public:
    // Global horizontal component
    GaussianVariable<double> Latitude;
    // Global vertical component
    GaussianVariable<double> Longitude;
    // Heading west of north (degrees)
    GaussianVariable<double> Heading;

    friend std::ostream &operator<< (std::ostream &stream, Position &s)
    {
        stream << s.Latitude.Value() << " (" << s.Latitude.Variance() << "), ";
        stream << s.Longitude.Value() << " (" << s.Longitude.Variance() << "), ";
        stream << s.Heading.Value() << " (" << s.Heading.Variance() << ")";
        return stream;
    }
};

class PositionTracker
{
public:
    PositionTracker();

    Position GetPosition();

private:
    Position UpdateWithMotion(Position S, Position Delta);
    Position UpdateWithMeasurement(Position S, Position Measurement);

    Position DeltaFromMotionCommand(MotorCommand cmd);
    Position MeasurementFromIMUData(IMUData data);

    Position _current_estimate;

    void OnGPSData(GPSData data);

    void OnMotionCommand(MotorCommand cmd);

    void OnIMUData(IMUData data);

public:
    LISTENER(PositionTracker, OnGPSData, GPSData)
    LISTENER(PositionTracker, OnMotionCommand, MotorCommand)
    LISTENER(PositionTracker, OnIMUData, IMUData)
};

#endif // POSITIONTRACKER_H
