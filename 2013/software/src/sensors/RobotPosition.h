#ifndef ROBOTPOSITION_H
#define ROBOTPOSITION_H

#include "DataStructures/GPSData.h"
#include "DataStructures/DataArray.h"
#include "DataStructures/DataPoint.hpp"
#include "DataStructures/IMUData.hpp"

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
        void push (IMUData newData);
        int update(GPSData);
        int update (IMUData);
        //Private functions being tested
        double avgSpeed(double end, double start);
        double avgHeading(double end, double start);
        void CatersianStep(int endingInd, double& x, double& y);
        void Cartesian2Polar(double x, double y, double& theta, double& r);
        double linInterp(double time, DataArray<DataPoint <double> >& attr);
        double rateOfChange(int ind, DataArray<DataPoint <double> >& attr);
        double latAtTime(double time);
        double longAtTime(double time);
        double speedAtTime(double time);
        double headingAtTime(double time);
        void latLongUpdate(double oldLat, double oldLong, double distance, double angle, double& newLat, double& newLong);
        void latLongCartesianUpdate(double oldLat, double oldLong, double deltaX, double deltaY, double& newLat, double& newLong);
        DataArray<DataPoint <double> >& Lat(void);
        DataArray<DataPoint <double> >& Long(void);
        DataArray<DataPoint <double> >& Heading(void);
        DataArray<DataPoint <double> >& Speed(void);
        GPSData IMU2GPS(IMUData in);
        double LatRateOfChange(double endInd);
        double LongRateOfChange(double endInd);
        void print(void);

    private:
        //double avgSpeed(double end, double start);
        //double avgHeading(double end, double start);
        double ptIntSpeed(int endingInd);
        double ptIntHeading(int endingInd);
        double ptAvgHeading(int endingInd);
        double predictSpeed(double time);
        double predictHeading(double time);
        double predictLat(double time);
        double predictLong(double time);
        DataArray<DataPoint <double> > _Lat;
        DataArray<DataPoint <double> >  _Long;
        DataArray<DataPoint <double> > _Speed;
        DataArray<DataPoint <double> >  _Heading;
        GPSAccuracy _Accuracy;
};

#endif // ROBOTPOSITION_H
