#ifndef TESTGPSUTILS_H
#define TESTGPSUTILS_H

#include <QtTest>
#include "common/utils/GPSUtils.h"
#include <iostream>
#include <iomanip>

class TestGPSUtils: public QObject
{
    Q_OBJECT

private Q_SLOTS:
    void testCase1()
    {
        using namespace std;

        double lat1 = 33.778402;
        double lon1 = -84.401301;
        double lat2 = 33.778430;
        double lon2 = -84.401215;

        double dist = GPSUtils::coordsToMeter(lat1,lon1,lat2,lon2);
        std::cout<<"Distance: "<<dist<<"m"<<endl;

        double deltaY = 7.9238;
        double deltaX = 3.2014;
        double newLat;
        double newLon;

        GPSUtils::latLongCartesianUpdate(lat1,lon1,deltaX,deltaY,newLat,newLon);

        std::cout<<std::setprecision(9)<<"Expected\t Lat: "<<lat2<<"   Lon"<<lon2<<endl;
        std::cout<<std::setprecision(9)<<"Was\t Lat: "<<newLat<<" Lon"<<newLon<<endl;
        double distBetween = GPSUtils::coordsToMeter(lat2,lon2,newLat,newLon);
        std::cout<<"Distance between expected and calculated is: "<<distBetween<<"m"<<endl;
    }
};



#endif // TESTGPSUTILS_H

