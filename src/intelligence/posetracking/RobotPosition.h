#ifndef ROBOTPOSITION_H
#define ROBOTPOSITION_H

#include <common/datastructures/GPSData.hpp>
#include <common/datastructures/DataArray.hpp>
#include <common/datastructures/DataPoint.hpp>
#include <common/datastructures/IMUData.hpp>
#include <common/datastructures/VisOdomData.hpp>
#include <hardware/sensors/IMU/IMU.h>
#include <hardware/sensors/gps/HemisphereA100GPS.h>
#include <intelligence/vision/2D/GrassOdometer/GrassOdometer.h>

#include <boost/thread.hpp>


const int millisPerSecond = 1000;

class RobotPosition
{
    public:
        RobotPosition();
        RobotPosition(IGVC::Sensors::GPS* gps, IMU* imu);
        LISTENER(RobotPosition, onNewGPSData, GPSData)
        LISTENER(RobotPosition, onNewIMUData, IMUData)
        LISTENER(RobotPosition, onNewVisOdomData, VisOdomData)
        void addOdometer(GrassOdometer* odo);
        int onNewGPSData(GPSData);
        int onNewIMUData(IMUData);
        int onNewVisOdomData(VisOdomData);
        virtual ~RobotPosition();
        double accel(int Index);
        double angVel(int Index);
        void push(GPSData newData);
        void push (IMUData newData);
        int update(GPSData);
        int update (IMUData);
        int update(VisOdomData);


        //Private functions being tested
        double avgSpeed(double end, double start);
        double avgHeading(double end, double start);
        double linInterp(double time, DataArray<DataPoint <double> >& attr);
        double rateOfChange(int ind, DataArray<DataPoint <double> >& attr);
        double latAtTime(double time);
        double longAtTime(double time);
        double speedAtTime(double time);
        double headingAtTime(double time);

        double currentLat();
        double currentLong();
        double currentHeading();
        double currentRoll();
        double currentPitch();
        double currentYaw();

        void latLongCartesianUpdate(double oldLat, double oldLong, double deltaX, double deltaY, double& newLat, double& newLong);

        GPSData VisOdom2GPS(VisOdomData in);

        void print(void);
        DataArray<DataPoint <double> >& Lat(void);
        DataArray<DataPoint <double> >& Long(void);
        DataArray<DataPoint <double> >& Heading(void);
        DataArray<DataPoint <double> >& Speed(void);
        bool try_lock();
        void lock();
        void unlock();

        DataArray<DataPoint <double> > _Lat;
        DataArray<DataPoint <double> >  _Long;
        DataArray<DataPoint <double> > _Speed;
        DataArray<DataPoint <double> > _Heading;
        DataArray<DataPoint <double> > _Roll;
        DataArray<DataPoint <double> > _Pitch;
        DataArray<DataPoint <double> > _Yaw;

    private:
        IGVC::Sensors::GPS* _GPS;
        IMU* _IMU;
        GrassOdometer* _Odom;
        double ptIntSpeed(int endingInd);
        double ptIntHeading(int endingInd);
        double ptAvgHeading(int endingInd);
        GPSAccuracy _Accuracy;
        boost::mutex StateMutex;
};

#endif // ROBOTPOSITION_H
