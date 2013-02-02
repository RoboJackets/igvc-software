#ifndef ROBOTPOSITION_H
#define ROBOTPOSITION_H

#include "DataStructures/GPSData.h"
#include "DataStructures/DataArray.h"
#include "DataStructures/DataPoint.hpp"

const int millisPerSecond = 1000;

class RobotPosition
{
    public:
        RobotPosition();
        void fuse(GPSData);
        //fuse(IMUData);
        //fuse(EncoderData);
        virtual ~RobotPosition();
        double accel(int Index);
        double angVel(int Index);
        void push(GPSData newData);


    private:
        double avgLat(long long end, long long start);
        double avgLong(long long end, long long start);
        double avgSpeed(long long end, long long start);
        double avgHeading(long long end, long long start);
        double ptIntLat(int endingInd); //Integral of value over the period between endingInd-1 and endingInd
        double ptIntLong(int endingInd);
        double ptIntSpeed(int endingInd);
        double ptIntHeading(int endingInd);

        DataArray<DataPoint <double> > Lat;
        DataArray<DataPoint <double> >  Long;
        DataArray<DataPoint <double> > Speed;
        DataArray<DataPoint <double> >  Heading;
        double LatVar;
        double longVar;
        double SpeedVar;
        double HeadingVar;
};

#endif // ROBOTPOSITION_H
