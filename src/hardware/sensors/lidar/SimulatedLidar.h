#ifndef SIMULATEDLIDAR_H
#define SIMULATEDLIDAR_H

#include <boost/thread.hpp>
#include "hardware/sensors/lidar/Lidar.h"

class SimulatedLidar : public Lidar
{
    public:
        SimulatedLidar();
        virtual ~SimulatedLidar();
        LidarState GetState();
        LidarState GetStateAtTime(timeval time);
        bool IsWorking();
        void loadFile(const char* path);
        void setDelay(int usec);

    protected:
    private:

    boost::thread _thread;

    bool _running;

    int _delay;

    void thread_run();

    LidarState _data;

    std::vector<std::string> StringSplit(const std::string &source, const char *delimiter = " ", bool keepEmpty = false);
};

#endif // SIMULATEDLIDAR_H
