#include <iostream>
#include <unistd.h>
#include <iomanip>
#include "SensorData.h"
#include "DataArray.h"
//#include <boost/circular_buffer.hpp>

#include "DataStructures/GPSData.h"
#include "RobotPosition.h"
#include "DataStructures/DataPoint.hpp"


int main()
{

    //GPSData dat(49.12, 0.2122);
    //std::cout << "Longitude is "<< dat.Long() << " Latitude is "<< dat.Lat() << " Time is " << dat.time() << std::endl;

    DataArray<GPSData> GPSArray(100);
    //GPSData dat(49.12, 0.2122);
    //GPSArray.push(dat);
    //GPSArray.push(GPSData(49.12, 0.2122, 11,1));
    //GPSArray.push(GPSData(49.12, 0.2122, 11,1));
    RobotPosition Mistii;
    Mistii.push(GPSData(49.12, 0.2122, 11,1));
    sleep(2);
    Mistii.push(GPSData(49.12, 0.2122, 11,11));
    sleep(2);
    Mistii.push(GPSData(49.12, 0.2122, 11,5));
    sleep(2);
    Mistii.push(GPSData(49.12, 0.2122, 11,7));


    std::cout<< "First speed: " << Mistii.Speed()[0].value() << ". Second Speed: " << Mistii.Speed()[1].value() << std::endl;
    std::cout << std::setprecision(5) << Mistii.accel(1) << std::endl;
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
