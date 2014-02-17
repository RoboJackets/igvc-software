#ifndef POSITIONTRACKER_H
#define POSITIONTRACKER_H

#include <common/utils/gaussianvariable.hpp>
#include <iostream>
#include <hardware/sensors/gps/GPS.hpp>
#include <hardware/actuators/motors/MotorDriver.hpp>
#include <hardware/sensors/IMU/IMU.h>
#include <common/events/Event.hpp>

/*!
 * \brief Data structure for robot's position.
 */
class Position
{
public:
    /** Global horizontal component */
    GaussianVariable<double> Latitude;
    /** Global vertical component */
    GaussianVariable<double> Longitude;
    /** Heading west of north (degrees) */
    GaussianVariable<double> Heading;

    friend std::ostream &operator<< (std::ostream &stream, Position &s)
    {
        stream << s.Latitude << " (" << s.Latitude.Variance << "), ";
        stream << s.Longitude << " (" << s.Longitude.Variance << "), ";
        stream << s.Heading<< " (" << s.Heading.Variance << ")";
        return stream;
    }

    bool operator == (const Position& other)
    {
        return  Latitude == other.Latitude &&
                Longitude == other.Longitude &&
                Heading == other.Heading;
    }

    bool operator != (const Position& other)
    {
        return !(operator ==(other));
    }

    bool operator == (const Position& other) const
    {
        return  Latitude == other.Latitude &&
                Longitude == other.Longitude &&
                Heading == other.Heading;
    }

    bool operator != (const Position& other) const
    {
        return !(operator ==(other));
    }
};

/*!
 * \brief Class for tracking robot's position.
 *
 *  Performs sensor fusion on motion and position sensors via kalman filtering.
 */
class PositionTracker
{
public:
    PositionTracker();

    Position GetPosition();

private:
    /*!
     * \brief UpdateWithMotion Accounts for the given motion predicition in the position estimate.
     * \param S The initial position, before motion
     * \param Delta The estimated change in position with the motion
     * \return The new estimate accounting for estimated motion
     */
    Position UpdateWithMotion(Position S, Position Delta);
    /*!
     * \brief UpdateWithMeasurement Accounts for the given measurement in the position estimate.
     * \param S The initial position estimate
     * \param Measurement The measured position
     * \return The new position estimate
     */
    Position UpdateWithMeasurement(Position S, Position Measurement);

    /*!
     * \brief DeltaFromMotionCommand Generates position delta estimate from motion command
     * \param cmd The motion command
     * \return The estimated change in position
     */
    Position DeltaFromMotionCommand(MotorCommand cmd);
    /*!
     * \brief MeasurementFromIMUData Generates measurement from IMU data
     * \param data The IMU data to use
     * \return The position measurement
     *
     * This method works by estimating a position change from the IMU data and adding that to the current position estimate.
     */
    Position MeasurementFromIMUData(IMUData data);

    Position _current_estimate;

    void OnGPSData(GPSData data);

    void OnMotionCommand(MotorCommand cmd);

    void OnIMUData(IMUData data);

    double prevIMUTime;

public:
    LISTENER(PositionTracker, OnGPSData, GPSData)
    LISTENER(PositionTracker, OnMotionCommand, MotorCommand)
    LISTENER(PositionTracker, OnIMUData, IMUData)
};

#endif // POSITIONTRACKER_H
