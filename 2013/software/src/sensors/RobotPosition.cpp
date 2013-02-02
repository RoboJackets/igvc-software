#include "RobotPosition.h"
#include <math.h>

RobotPosition::RobotPosition() : Lat(100), Long(100), Speed(100), Heading(100)
{
    //ctor
}

RobotPosition::~RobotPosition()
{
    //dtor
}

void RobotPosition::push(GPSData newData)
{
    Lat.push(newData.Lat());
    Long.push(newData.Long());
    Speed.push(newData.Speed());
    Heading.push(newData.Heading());
}

double RobotPosition::accel(int Index)
{
    int millisPerSecond = 1000;
    if (Index==0)
    {
        return 0;
    }
    else
    {
        return millisPerSecond*(Speed[Index].value()-Speed[Index-1].value())/(Speed[Index].time()-Speed[Index-1].time());
    }
}


double RobotPosition::angVel(int Index)
{

    if (Index==0)
    {
        return 0;
    }
    else
    {
        return millisPerSecond*(Heading[Index].value()-Heading[Index-1].value())/(Heading[Index].time()-Heading[Index-1].time());
    }
}

double RobotPosition::ptIntSpeed(int endingInd)
{
    //TODO Account for when an indice lower than 2 is pased
    int dti = Speed[endingInd].time() - Speed[endingInd-1].time();
    int dtiMinus = Speed[endingInd-1].time() - Speed[endingInd-2].time();
    int dt = dti + dtiMinus;
    double A = (millisPerSecond*(accel(endingInd)-accel(endingInd-1))/dt)/2;
    double B = accel(endingInd-1);
    double C = Speed[endingInd-1].value();
    int integral = A/3*pow((dti/millisPerSecond),3) + (B/2) * pow((dti/millisPerSecond),2) + C*dti/millisPerSecond;
    return integral;
}

double RobotPosition::avgSpeed(long long end, long long start)
{
    int startInd = Speed.firstIndBefore(start);
    int endInd = Speed.firstIndBefore(end);
    double integral=0;
    int i;
    long long dt;
    for (i=startInd+1;i<=endInd;i++)
    {
        integral+=ptIntSpeed(i);
    }
    dt = Speed[endInd].time() - Speed[startInd].time();
    return millisPerSecond*integral/dt;




}
