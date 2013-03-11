#ifndef ROBOTPOSITION_H
#define ROBOTPOSITION_H

#include "DataStructures/GPSData.h"
#include "DataStructures/DataArray.h"
#include "DataStructures/DataPoint.hpp"
#include "DataStructures/IMUData.hpp"
#include "IMU/IMU.h"
#include "GPS/HemisphereA100GPS.h"

#include <boost/thread.hpp>


const int millisPerSecond = 1000;

class RobotPosition
{
    public:
        RobotPosition();
        //RobotPosition(IGVC::Sensors::GPS* gps, IMU*);
        //LISTENER(RobotPosition, onNewGPSData, GPSData);
        //LISTENER(RobotPosition, onNewIMUData, IMUData);
        int onNewGPSData(GPSData);
        int onNewIMUData(IMUData);
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
        double linInterp(double time, DataArray<DataPoint <double> >& attr);
        double rateOfChange(int ind, DataArray<DataPoint <double> >& attr);
        double latAtTime(double time);
        double longAtTime(double time);
        double speedAtTime(double time);
        double headingAtTime(double time);
        void latLongCartesianUpdate(double oldLat, double oldLong, double deltaX, double deltaY, double& newLat, double& newLong);
        GPSData IMU2GPS(IMUData in);
        void print(void);
        DataArray<DataPoint <double> >& Lat(void);
        DataArray<DataPoint <double> >& Long(void);
        DataArray<DataPoint <double> >& Heading(void);
        DataArray<DataPoint <double> >& Speed(void);
        bool try_lock();
        void lock();
        void unlock();

    private:
        double ptIntSpeed(int endingInd);
        double ptIntHeading(int endingInd);
        double ptAvgHeading(int endingInd);
        DataArray<DataPoint <double> > _Lat;
        DataArray<DataPoint <double> >  _Long;
        DataArray<DataPoint <double> > _Speed;
        DataArray<DataPoint <double> >  _Heading;
        GPSAccuracy _Accuracy;
        boost::mutex StateMutex;

};

#endif // ROBOTPOSITION_H
