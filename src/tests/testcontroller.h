#ifndef TESTCONTROLLER_H
#define TESTCONTROLLER_H

#include <QtTest>
#include "intelligence/controller/controller.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <hardware/sensors/gps/simulatedgps.h>

class TestController: public QObject
{
    Q_OBJECT

    private Q_SLOTS:

    void test()
    {
        GPSWaypointSource *s = new GPSWaypointSource("/home/alchaussee/Desktop/properlyFormedGPSData.txt");
        GPS *gps = new SimulatedGPS("/home/alchaussee/Desktop/properlyFormedGPSData.txt");

        Controller con (s, gps);
        GPSData d = GPSData(33.7871928,-84.4063265);
        //QCOMPARE(d.Lat(), con.getCurrentWaypoint().Lat());
        //QCOMPARE(d.Long(), con.getCurrentWaypoint().Long());


         std::cout << "CON "<<con.getCurrentWaypoint().Lat() << " " << con.getCurrentWaypoint().Long() << std::endl;
         //std::cout << "GPS "<<gps->GetState().Lat() << " " << gps->GetState().Long() << std::endl;
         QTest::qWait(30000);


    }
};


#endif // TESTCONTROLLER_H
