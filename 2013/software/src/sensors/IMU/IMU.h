#ifndef IMU_H
#define IMU_H

#include "events/Event.hpp"

class IMU
{
    public:
        IMU();
        Event<IMUData>onNewData;


    private:

};



#endif //IMU_H
