#include "RobotPosition.h"
#include <math.h>
#include <cmath>
#include <common/utils/timing.h>
#include <iostream>
#include "common/utils/GPSUtils.h"

#define deg2rad(a) a/180*M_PI
#define rad2deg(a) a*180/M_PI


/**
Odometer is not properly initialized due to circular dependency, addOdometer should be called once odometer is initialized using position
**/
RobotPosition::RobotPosition(IGVC::Sensors::GPS* gps, IMU* imu) : LonNewGPSData(this), LonNewIMUData(this), LonNewVisOdomData(this),
 _GPS(gps), _IMU(imu), _Odom(0), _Lat(100), _Long(100), _Speed(100), _Heading(100), _Roll(100), _Pitch(100), _Yaw(100), _Accuracy(100, 100,100, 100)
{
    if(gps)
        gps->onNewData += &LonNewGPSData;
    if(imu)
        imu->onNewData += &LonNewIMUData;
}

RobotPosition::RobotPosition() : LonNewGPSData(this), LonNewIMUData(this), LonNewVisOdomData(this), _GPS(0), _IMU(0), _Lat(100),
  _Long(100), _Speed(100), _Heading(100), _Roll(100), _Pitch(100), _Yaw(100), _Accuracy(100, 100,100, 100)
{

}

void RobotPosition::addOdometer(GrassOdometer* odo)
{
  _Odom = odo;
  _Odom->onNewData += &LonNewVisOdomData;
}


RobotPosition::~RobotPosition()
{
  if (_GPS != 0)
  {
    _GPS->onNewData -= &LonNewGPSData;
  }
  if (_IMU != 0)
  {
    _IMU-> onNewData -= &LonNewIMUData;
  }
  if (_Odom != 0)
    _Odom-> onNewData -= &LonNewVisOdomData;
}
/**
User must get mutex lock prior to use
**/
void RobotPosition::push(GPSData newData)
{
    double time = seconds_since_IGVCpoch();
    _Lat.push(DataPoint<double>(newData.Lat(), time));
    _Long.push(DataPoint<double>(newData.Long(), time));
    _Speed.push(DataPoint<double>(newData.Speed(), time));
    _Heading.push(DataPoint<double>(newData.Heading(), time));

    //combine accuracies
}

int RobotPosition::onNewGPSData(GPSData newData)
{
  //return update(newData);
  push(newData);
  return 0;
}


int RobotPosition::onNewIMUData(IMUData newData)
{
  return update(newData);
}

int RobotPosition::onNewVisOdomData(VisOdomData newData)
{
  return update(newData);
}

/**
Determines the acceleration of the robot between two indices.
User must get mutex lock prior to use
**/
double RobotPosition::accel(int Index)
{
    double dT;
    double dV;
    if (Index==(_Speed.size()-1))
    {
        return 0;
    }
    else
    {
        dV = _Speed[Index].value()-_Speed[Index+1].value();
        dT = _Speed[Index].time()-_Speed[Index+1].time();
        return dV/dT;
    }
}
/**
User must get mutex lock prior to use
**/
double RobotPosition::angVel(int Index)
{
    double dTheta;
    double dT;

    if (Index==(_Heading.size()-1))
    {
        return 0;
    }
    else
    {
        dTheta = (_Heading[Index].value()-_Heading[Index+1].value());
        dT = (_Heading[Index].time()-_Heading[Index+1].time());
        return dTheta/dT;
    }
}

/***
Returns the Integral of the speed(i.e. distance) over the period between endingInd and endingInd+1
User must get mutex lock prior to use
***/

double RobotPosition::ptIntSpeed(int endingInd)
{
    //TODO Account for when an indice lower than 2 is pased
    int dti = _Speed[endingInd].time() - _Speed[endingInd+1].time();
    int dtiMinus = _Speed[endingInd+1].time() - _Speed[endingInd+2].time();
    int dt = dti + dtiMinus;
    double A = ((accel(endingInd)-accel(endingInd+1))/dt)/2;
    double B = accel(endingInd+1);
    double C = _Speed[endingInd+1].value();
    double integral = A/3*pow((dti),3) + (B/2) * pow((dti),2) + C*dti;
    return integral;
}

/**
User must get mutex lock prior to use
**/

double RobotPosition::avgSpeed(double end, double start)
{
    int startInd = _Speed.firstIndBefore(start);
    int endInd = _Speed.firstIndAfter(end);
    double integral=0;
    int i;
    double dT;
    for (i=(startInd-1); i>=endInd; i--)
    {
        integral+=ptIntSpeed(i);
    }
    dT = _Speed[endInd].time() - _Speed[startInd].time();
    return integral/dT;
}


/**
Thread safe, gets lock before performing work
**/
//TODO simplify this by writing a method to do probabibalistic combination
int RobotPosition::update(GPSData newData)
{

    while(!StateMutex.try_lock())
    {
    }

    double newTime = newData.time();


    if (_Lat.size()<3) //if there aren't enough points for a full data model, just use it as new data
    {
      push(newData);
    }
    else if (newData.time() < _Lat[0].time()) // If data is behind current knowledge, disregard it
    {
      return -1;
    }
    else //Otherwise, using Kalmann filtering
    {
    double predLat = latAtTime(newTime);
    double predLong = longAtTime(newTime);
    double predSpeed = speedAtTime(newTime);
    double predHeading = headingAtTime(newTime);

     double newLatVar = _Accuracy.LatVar() * (1+abs(newData.Lat()-_Lat[0].value()));
     double newLongVar = _Accuracy.LongVar()  * (1+abs(newData.Long()-_Long[0].value()));
     double newHeadingVar = _Accuracy.HeadingVar() * (1+abs(newData.Heading()-_Heading[0].value()));
     double newSpeedVar = _Accuracy.SpeedVar() * (1+abs(newData.Speed()-_Speed[0].value()));


     double newLat = (predLat*newData.LatVar() + newData.Lat()*newLatVar)/(newData.LatVar()+newLatVar);
     double newLong = (predLong*newData.LongVar() + newData.Long()*newLongVar)/(newData.LongVar()+newLongVar);
     double newHeading = (predHeading*newData.HeadingVar() + newData.Heading()*newHeadingVar)/(newData.HeadingVar()+newHeadingVar);
     double newSpeed = (predSpeed*newData.SpeedVar() + newData.Speed()*newSpeedVar)/(newData.SpeedVar()+newSpeedVar);

     _Lat.push(DataPoint<double>(newLat, newData.time()));
     _Long.push(DataPoint<double>(newLong, newData.time()));
     _Heading.push(DataPoint<double>(newHeading, newData.time()));
     _Speed.push(DataPoint<double>(newSpeed, newData.time()));

    _Accuracy.LatVar(_Accuracy.LatVar()*newData.Accuracy().LatVar()/(newData.Accuracy().LatVar()+newLatVar));
    _Accuracy.LongVar(_Accuracy.LongVar()*newData.Accuracy().LongVar()/(newData.Accuracy().LongVar()+newLongVar));
    _Accuracy.HeadingVar(_Accuracy.HeadingVar()*newData.Accuracy().HeadingVar()/(newData.Accuracy().HeadingVar()+newHeadingVar));
    _Accuracy.SpeedVar(_Accuracy.SpeedVar()*newData.Accuracy().SpeedVar()/(newData.Accuracy().SpeedVar()+newSpeedVar));

    std::cout << "Prediction was " << predSpeed << ". Measurement was " << newSpeed<< "." << std::endl;
  }

  StateMutex.unlock();
  return 0;
}

/**
Updates robot position information using IMU data
**/
int RobotPosition::update(IMUData newData)
{

  double time = newData.time();
  _Roll.push(DataPoint<double>(newData.Roll, time));
  _Pitch.push(DataPoint<double>(newData.Pitch, time));
  _Yaw.push(DataPoint<double>(newData.Yaw, time));
  return 0;
}


int RobotPosition::update(VisOdomData newData)
{
  if (_Lat.size()<3) //Ensures there is absolutele information position before we try to use relative to update it.
  {
    return -1;
  }
  else
  {
    GPSData newForm = VisOdom2GPS(newData);
    return update(newForm);
  }
}

GPSData RobotPosition::VisOdom2GPS(VisOdomData in)
{
  double lastMeasurementTime = in.time()-in.deltaTime();
  double oldLat = latAtTime(lastMeasurementTime);
  double oldLong = longAtTime(lastMeasurementTime);
  double newLat;
  double newLong;
  latLongCartesianUpdate(oldLat, oldLong, in.deltaX(), in.deltaY(), newLat, newLong);
  GPSData newForm(newLat, newLong, in.Heading(), in.Speed(), in.time());
  return newForm;
}

double RobotPosition::linInterp(double time, DataArray<DataPoint <double> >& attr)
{
    int previous = attr.firstIndBefore(time);
    if (previous == 0) // If the time is after our latest information
    {
        return attr[0].value();
    }

    else //Do linear interpolation
    {
        double last = attr[previous].value();
        double slope = rateOfChange((previous-1), attr);
        double dT =  time-attr[previous].time();
        return last + (slope*dT);
    }
    //TODO add code to ensure that there are at least two values available or state as assumption
    //There may need to be code written for the case in which
}

double RobotPosition::latAtTime(double time)
{
    return linInterp(time, _Lat);
}

double RobotPosition::longAtTime(double time)
{
    return linInterp(time, _Long);
}

double RobotPosition::headingAtTime(double time)
{
    return linInterp(time, _Heading);
}

double RobotPosition::speedAtTime(double time)
{
    return linInterp(time, _Speed);
}

double RobotPosition::currentRoll()
{
    return _Roll[0].value();
}

double RobotPosition::currentPitch()
{
    return _Pitch[0].value();
}

double RobotPosition::currentYaw()
{
    return _Yaw[0].value();
}

double RobotPosition::currentLat()
{
    return _Lat[0].value();
}

double RobotPosition::currentLong()
{
    return _Long[0].value();
}

double RobotPosition::currentHeading()
{
    return _Heading[0].value();
}

//Gives the linear rate of change between two indices of one of the data structures
double RobotPosition::rateOfChange(int ind, DataArray<DataPoint <double> >& attr)
{
    double newVal = attr[ind].value();
    double oldVal = attr[ind+1].value();
    double dT = attr[ind].time()-attr[ind+1].time();
    return (newVal-oldVal)/dT;
}

//Taken from http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
void RobotPosition::latLongCartesianUpdate(double oldLat, double oldLong, double deltaX, double deltaY, double& newLat, double& newLong)
{
    GPSUtils::latLongCartesianUpdate(oldLat, oldLong, deltaX, deltaY, newLat, newLong);
}

void RobotPosition::print(void) {
std::cout << "Lat: " << _Lat[0].value() << ".Long: " << _Long[0].value() << ".Heading: " << _Heading[0].value() << "Speed: " << _Speed[0].value() << ".";
//TODO add print method to acccuracy and call it here
}

DataArray<DataPoint <double> >& RobotPosition::Lat(void)
{
    return _Lat;
}

DataArray<DataPoint <double> >& RobotPosition::Long(void)
{
    return _Long;
}

bool RobotPosition::try_lock()
{
    return StateMutex.try_lock();
}

void RobotPosition::lock()
{
    StateMutex.lock();
    return;
}

void RobotPosition::unlock()
{
    StateMutex.unlock();
    return;
}
