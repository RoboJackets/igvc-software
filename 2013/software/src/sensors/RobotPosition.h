#ifndef ROBOTPOSITION_H
#define ROBOTPOSITION_H

#include "DataStructures/GPSData.h"

class RobotPosition
{
    public:
        RobotPosition();
        void fuse(GPSData);
        //fuse(IMUData);
        //fuse(EncoderData);
        virtual ~RobotPosition();


    private:
        DataArray<double> Lat(100);
        DataArray<double> Long(100);
        DataArray<double> Speed(100);
        DataArray<double> Heading(100);

};

#endif // ROBOTPOSITION_H
