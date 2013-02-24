#include "RobotPosition.h"
#include <math.h>
#include <cmath>
#include "timing.h"
#include <iostream>

#define deg2rad(a) a/180*M_PI
#define rad2deg(a) a*M_PI/180

RobotPosition::RobotPosition() : _Lat(100), _Long(100), _Speed(100), _Heading(100), _Accuracy(100, 100,100, 100)
{
    //ctor
}

RobotPosition::~RobotPosition()
{
    //dtor
}

void RobotPosition::push(GPSData newData)
{
    double time = seconds_since_IGVCpoch();
    _Lat.push(DataPoint<double>(newData.Lat(), time));
    _Long.push(DataPoint<double>(newData.Long(), time));
    _Speed.push(DataPoint<double>(newData.Speed(), time));
    _Heading.push(DataPoint<double>(newData.Heading(), time));
    //combine accuracies
}


/**
Determines the acceleration of the robot between two indices. The end
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

**/

double RobotPosition::ptIntHeading(int endingInd)
{
    //TODO Account for when an indice lower than 2 is pased
    int dti = _Heading[endingInd].time() - _Heading[endingInd+1].time();
    int dtiMinus = _Heading[endingInd+1].time() - _Heading[endingInd+2].time();
    int dt = dti + dtiMinus;
    double A = ((angVel(endingInd)-angVel(endingInd+1))/dt)/2;
    double B = angVel(endingInd+1);
    double C = _Heading[endingInd+1].value();
    double integral = A/3*pow((dti),3) + (B/2) * pow((dti),2) + C*dti;
    return integral;
}

double RobotPosition::ptAvgHeading(int endingInd)
{
    double integral = ptIntHeading(endingInd);
    int dT = _Heading[endingInd].time() - _Heading[endingInd+1].time();
    return integral/dT;
}

void RobotPosition::CatersianStep(int endingInd, double& x, double& y)
{
    x = ptIntSpeed(endingInd)*cos(ptAvgHeading(endingInd));
    y = ptIntSpeed(endingInd)*cos(ptAvgHeading(endingInd));
}

void RobotPosition::Cartesian2Polar(double x, double y, double& theta, double& r)
{
    //inputs reversed from normal usage because theta is complement of angle
    theta = atan2(x,y);
    r = sqrt(pow(x,2)+pow(y,2));
}

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

//TODO simplify this by writing a method to do probabibalistic combination
int RobotPosition::update(GPSData newData)
{
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

    return 0;
}


int RobotPosition::update(IMUData newData)
{
    if (_Lat.size()<3) //Ensures there is absolutele information position before we try to use relative to update it.
    {
        return -1;
    }
     else
     {
        GPSData newForm = IMU2GPS(newData);
        return update(newForm);
     }
}

GPSData RobotPosition::IMU2GPS(IMUData in)
{
    double lastMeasurementTime = in.time()-in.deltaTime();
    double offsetHeading;
    double offsetSpeed;
    Cartesian2Polar(in.deltaX(), in.deltaY(), offsetHeading, offsetSpeed);
    double oldLat = latAtTime(lastMeasurementTime);
    double oldLong = longAtTime(lastMeasurementTime);
    double newLat;
    double newLong;
    latLongCartesianUpdate(oldLat, oldLong, in.deltaX(), in.deltaY(), newLat, newLong);
    GPSData newForm((double)newLat, (double)newLong, in.Heading(), in.Speed(), in.time());
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
    newLat = oldLat + deltaY/111,111;
    newLong = oldLong + deltaX/(111,111*cos(oldLat));
}

void RobotPosition::print(void) {
std::cout << "Lat: " << _Lat[0].value() << ".Long: " << _Long[0].value() << ".Heading: " << _Heading[0].value() << "Speed: " << _Speed[0].value() << ".";
//TODO add print method to acccuracy and call it here
}

DataArray<DataPoint <double> >& RobotPosition::Lat(void)
{
    return _Lat;
}
