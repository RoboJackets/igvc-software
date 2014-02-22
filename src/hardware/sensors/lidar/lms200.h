#ifndef LMS200_H
#define LMS200_H

#include "Lidar.h"
#include <sicklms-1.0/SickLMS.hh>
#include <boost/thread.hpp>

class LMS200 : public Lidar
{
public:
    LMS200();
    ~LMS200();

    LidarState GetState();
    LidarState GetStateAtTime(timeval);

    bool IsWorking();

private:
    SickToolbox::SickLMS _device;

    boost::thread _thread;
    bool _thread_running;

    void thread_run();
};

#endif // LMS200_H
