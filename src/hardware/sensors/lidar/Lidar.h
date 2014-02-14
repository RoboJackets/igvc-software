/*! \file Lidar.h
 * \date Created on: Jan 22, 2012
 * \author Alexander Huynh, Matthew Barulic
 */

#ifndef LIDAR_H
#define LIDAR_H

#include <stdint.h>
#include <time.h>
#include "common/events/Event.hpp"

class LidarPoint
{
    public:
        LidarPoint()
        {
            valid = false;
            angle = 0;
            raw = 0;
            distance = 0;
            intensity = 0;
        }

        /*! \brief If false, the other members are not meaningful */
        bool valid;

        /*! \brief Angle in radians counterclockwise from right, with the LEDs pointing forward.*/
        float angle;

        /*! \brief Raw distance */
        uint16_t raw;

        /*! \brief Distance in meters*/
        float distance;

        /*! \brief Intensity of return, unknown units */
        uint8_t intensity;
};

/*!
 * \brief A struct that represents a data packet from the Lidar device.
 */
struct LidarState
{
    timeval timestamp;
    LidarPoint points[1024];
};

/*!
 * \brief Interface for Lidars.
 * \headerfile Lidar.h <hardware/sensors/lidar/Lidar.h>
 */
class Lidar
{
public:
    virtual ~Lidar() { }

    /*!
     * \brief Returns the most recent state acquired from the Lidar.
     */
    virtual LidarState GetState() = 0;

    /*!
     * \brief Returns the LidarState with the given timestamp.
     */
    virtual LidarState GetStateAtTime(timeval time) = 0;

    virtual bool IsWorking() = 0;

    Event<LidarState> onNewData;
    Event<void*> onDeviceFailure;
    Event<void*> onDataExpiration;
};

#endif // LIDAR_H
