#ifndef SIMULATEDLIDAR_H
#define SIMULATEDLIDAR_H

#include <boost/thread.hpp>
#include "sensors/lidar/Lidar.h"

namespace IGVC
{
    namespace Sensors
    {
        class SimulatedLidar : public Lidar
        {
            public:
                SimulatedLidar();
                virtual ~SimulatedLidar();
                LidarState GetState();
                LidarState GetStateAtTime(timeval time);
                bool StateIsAvailable();
            protected:
            private:

            boost::thread _thread;

            bool _running;

            void thread_run();
        };
    }
}

#endif // SIMULATEDLIDAR_H
