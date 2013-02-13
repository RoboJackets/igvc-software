/*
 * Lidar.h
 *
 *  Created on: Jan 22, 2012
 *      Author: Alexander Huynh
Copypasta from: Matthew Barulic
 */

#ifndef LIDAR_H
#define LIDAR_H

#include <stdint.h>
#include <time.h>
#include "events/Event.hpp"

namespace IGVC {
namespace Sensors {

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

            // If false, the other members are not meaningful
            bool valid;

            // Angle in radians counterclockwise from right, with the LEDs pointing forward.
            float angle;

            // Raw distance
            uint8_t raw;

            // Distance in meters
            float distance;

            // Intensity of return, unknown units
            uint8_t intensity;
    };

    /*
     * A struct that represents a data packet from the Lidar device.
     */
    struct LidarState
    {
        timeval timestamp;
        LidarPoint points[1024];
    };

/*
 * Interface for Lidars.
 */
class Lidar
{
public:
    virtual ~Lidar() { }

    /*
     * Returns the most recent state acquired from the Lidar.
     */
    virtual LidarState GetState() = 0;

    /*
     * Returns the LidarState with the given timestamp.
     * Throws an error if no such LidarState exists in the buffer.
     */
    virtual LidarState GetStateAtTime(timeval time) = 0;

    /*
     * Return true if there is at least one state in the buffer.
     */
    virtual bool StateIsAvailable() = 0;

    Event<LidarState> onNewData;
    Event<void*> onDeviceFailure;
    Event<void*> onDataExpiration;
};


} /* namespace Sensors */
} /* namespace IGVC */
#endif // LIDAR_H
