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
        std::shared_ptr<GPSWaypointSource> s = std::shared_ptr<GPSWaypointSource>(new GPSWaypointSource("../test_data/GPSFileReader-testData/Successful/properlyFormedGPSData.txt"));
        std::shared_ptr<GPS> gps = std::shared_ptr<GPS>(new SimulatedGPS("../test_data/GPSFileReader-testData/Successful/properlyFormedGPSData.txt"));

        Controller con (s, gps);
        GPSData d = GPSData(33.7871928,-84.4063265);
        //QCOMPARE(d.Lat(), con.getCurrentWaypoint().Lat());
        //QCOMPARE(d.Long(), con.getCurrentWaypoint().Long());


         //std::cout << "CON "<<con.getCurrentWaypoint().Lat() << " " << con.getCurrentWaypoint().Long() << std::endl;
         //std::cout << "GPS "<<gps->GetState().Lat() << " " << gps->GetState().Long() << std::endl;
         QTest::qWait(30000);


    }
};


#endif // TESTCONTROLLER_H
