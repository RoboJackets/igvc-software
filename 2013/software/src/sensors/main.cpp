#include <iostream>
#include <unistd.h>
#include <iomanip>
#include "SensorData.h"
#include "DataArray.h"
#include "timing.h"
//#include <boost/circular_buffer.hpp>

#include "DataStructures/GPSData.h"
#include "RobotPosition.h"
#include "DataStructures/DataPoint.hpp"

GPSAccuracy GPSData::NAV200Default = GPSAccuracy(.0001, .0001, 3, 0.01);
GPSAccuracy IMUData::ArduPilotDefault= GPSAccuracy(.001, .001, 1, .1);

int main()
{

    //GPSData dat(49.12, 0.2122);
    //std::cout << "Longitude is "<< dat.Long() << " Latitude is "<< dat.Lat() << " Time is " << dat.time() << std::endl;

    //DataArray<GPSData> GPSArray(100);
    //GPSData dat(49.12, 0.2122);
    //GPSArray.push(dat);
    //GPSArray.push(GPSData(49.12, 0.2122, 11,1));
    //GPSArray.push(GPSData(49.12, 0.2122, 11,1));


    double currTime = seconds_since_IGVCpoch();

    RobotPosition Mistii;
    Mistii.update(GPSData(49.12, 0.2122, 11,1, currTime+=2));
    std::cout << "ohai";
    Mistii.update(GPSData(49.12, 0.2122, 10,3, currTime+=2));
    std::cout << "ohai";
    Mistii.update(GPSData(49.12, 0.2122, 11,3, currTime+=2));
    std::cout << "ohai";
    Mistii.update(GPSData(49.12, 0.2122, 11,5, currTime+=2));
    std::cout << "ohai";
    Mistii.update(GPSData(49.32, 0.2122, 13,7, currTime+=2));
    Mistii.update(GPSData(49.32, 0.2122, 13,7, currTime+=2));
    Mistii.update(GPSData(49.32, 0.2122, 13,7, currTime+=2));
    std::cout << "ohai";

    Mistii.print();


    //std::cout<< "First speed: " << Mistii.Speed()[0].value() << ". Second Speed: " << Mistii.Speed()[1].value() << std::endl;

    //std::cout << "Average Speed over Period was" << Mistii.ptIntSpeed(0)/(Mistii.Speed()[0].time()-Mistii.Speed()[1].time());

    //std::cout << "Acceleration was" << Mistii.accel(0) << "      Should be 1";

    //std::cout << Mistii.latAtTime(Mistii.Lat()[1].time()+1);
    //std::cout << std::setprecision(5) << Mistii.accel(1) << std::endl;

    //std::cout << std::setprecision(4) << Mistii.angVel(1);
   // std::cout << GPSArray[0].Lat() <<  " Should be 49.12"<< std::endl;

    //std::cout << GPSArray.size() << std::endl;


    /*
    DataPoint<double> that(15.2);
    DataArray<DataPoint <double> > Lat(100);
    Lat.push(that);

    std::cout << Lat[0].time()<< std::endl;
    //std::cout << that.value() << std::endl;
    */

}
