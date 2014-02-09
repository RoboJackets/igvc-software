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

        double lat1 =  33.787295;
        double lon1 = -84.406323;
        double lat2 =  33.787301;
        double lon2 = -84.406185;

        //double lat1 =     30.855795;
        //double lon1 = -97.331163;
        //double lat2 =      30.856040;
        //double lon2 = -97.331447;

        double dist = GPSUtils::coordsToMeter(lat1,lon1,lat2,lon2);
        //std::cout<<"Distance(m): "<<dist<<endl;

        double newLat;
        double newLon;

        double dy = lat2 - lat1;
        double dx = cos(M_PI/180*lat1)*(lon2 - lon1);
        double angle = atan2(dy, dx);

        std::cout<<"Angle(deg): "<<angle * (180/M_PI)<<endl;

        //GPSUtils::latLongCartesianUpdate(lat1,lon1,deltaX,deltaY,newLat,newLon);
        GPSUtils::coordAfterMotion(lat1,lon1,dist,angle+(270*(M_PI/180.0)),newLat,newLon);
        double distBetween = GPSUtils::coordsToMeter(lat2,lon2,newLat,newLon);

        std::cout<<std::setprecision(9)<<"Expected\t Lat: "<<lat2<<"   Lon"<<lon2<<"   Dist: "<<dist<<endl;
        std::cout<<std::setprecision(9)<<"Test\t Lat: "<<newLat<<"  Lon"<<newLon<<"  Dist: "<<distBetween<<endl;
    }

    void testCoordsToMetricXY()
    {
        using namespace std;

        double lat1 =  33.787371;
        double lon1 = -84.406211;
        double lat2 =  33.787372;
        double lon2 = -84.406183;

        double dX = 0;
        double dY = 0;

        GPSUtils::coordsToMetricXY(lat1, lon1, lat2, lon2, dX, dY);

        std::cout << dX << ", " << dY << std::endl;
    }
};



#endif // TESTGPSUTILS_H

