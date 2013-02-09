#include "RobotPosition.h"
#include <math.h>
#include "timing.h"

RobotPosition::RobotPosition() : _Lat(100), _Long(100), _Speed(100), _Heading(100)
{
    //ctor
}

RobotPosition::~RobotPosition()
{
    //dtor
}


DataArray<DataPoint <double> > RobotPosition::Speed(void)
{
    return _Speed;
}


void RobotPosition::push(GPSData newData)
{
    double time = seconds_since_IGVCpoch();
    _Lat.push(DataPoint<double>(newData.Lat(), time));
    _Long.push(DataPoint<double>(newData.Long(), time));
    _Speed.push(DataPoint<double>(newData.Speed(), time));
    _Heading.push(DataPoint<double>(newData.Heading(), time));
}


/**
Determines the acceleration of the robot between two indices. The end
**/
double RobotPosition::accel(int Index)
{
    double dT;
    double dV;
    if (Index==0)
    {
        return 0;
    }
    else
    {
        dV = _Speed[Index].value()-_Speed[Index-1].value();
        dT = _Speed[Index].time()-_Speed[Index-1].time();
        return dV/dT;
    }
}


double RobotPosition::angVel(int Index)
{
    double dTheta;
    double dT;

    if (Index==0)
    {
        return 0;
    }
    else
    {
        dTheta = (_Heading[Index].value()-_Heading[Index-1].value());
        dT = (_Heading[Index].time()-_Heading[Index-1].time());
        return dTheta/dT;
    }
}




//TODO MAKE SURE THIS GUY WAS CHANGED CORRECTLY FOR
double RobotPosition::ptIntSpeed(int endingInd)
{
    //TODO Account for when an indice lower than 2 is pased
    int dti = _Speed[endingInd].time() - _Speed[endingInd-1].time();
    int dtiMinus = _Speed[endingInd-1].time() - _Speed[endingInd-2].time();
    int dt = dti + dtiMinus;
    double A = ((accel(endingInd)-accel(endingInd-1))/dt)/2;
    double B = accel(endingInd-1);
    double C = _Speed[endingInd-1].value();
    int integral = A/3*pow((dti),3) + (B/2) * pow((dti),2) + C*dti;
    return integral;
}

double RobotPosition::avgSpeed(long long end, long long start)
{
    int startInd = _Speed.firstIndBefore(start);
    int endInd = _Speed.firstIndBefore(end);
    double integral=0;
    int i;
    long long dt;
    for (i=startInd+1;i<=endInd;i++)
    {
        integral+=ptIntSpeed(i);
    }
    dt = _Speed[endInd].time() - _Speed[startInd].time();
    return millisPerSecond*integral/dt;
}
