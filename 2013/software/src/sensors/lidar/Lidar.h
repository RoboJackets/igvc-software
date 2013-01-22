#ifndef LIDAR_H
#define LIDAR_H

#include <time.h>
#include "events/EventGenerator.cpp"

namespace IGVC {
namespace Sensors {

//class Lidar
//{
    //public:
    class Point
    {
        public:
            Point()
            {
                valid = false;
                distance = 0;
                intensity = 0;
            }

            // If false, the other members are not meaningful
            bool valid;

            // Angle in radians counterclockwise from right, with the LEDs pointing forward.
            float angle;

            // Raw distance
            int raw;

            // Distance in meters
            float distance;

            // Intensity of return, unknown units
            int intensity;
    };
    static const int Num_Points = 1024;

    //Point pnt[Num_Points];

    class LidarListener
    {
        public:
            virtual void onNewStateAvailable(void* state) = 0;
    };

    /*
     * Interface for GPS devices.
     */
    class Lidar : public EventGenerator<LidarListener>
    {
    public:
        virtual ~Lidar() { }

        /*
         * Returns the most recent state acquired from the GPS device.
         */
        //virtual LidarState GetState() = 0;

        /*
         * Returns the LidarState with the given timestamp.
         * Throws an error if no such LidarState exists in the buffer.
         */
        //virtual LidarState GetStateAtTime(timeval time) = 0;

        /*
         * Return true if there is at least one state in the buffer.
         */
        virtual bool StateIsAvailable() = 0;

};

//};

} /* namespace Sensors */
} /* namespace IGVC */
#endif // LIDAR_H
