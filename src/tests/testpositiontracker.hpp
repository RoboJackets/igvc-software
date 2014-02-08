#ifndef TESTPOSITIONTRACKER_HPP
#define TESTPOSITIONTRACKER_HPP

#include <QTest>
#include <math.h>
#include <iostream>
#include <intelligence/posetracking/positiontracker.h>

class TestPositionTracker: public QObject
{
    Q_OBJECT
private Q_SLOTS:
    void testUpdateFromGPS()
    {
        PositionTracker tracker;
        Event<GPSData> testEvent;
        testEvent += &tracker.LOnGPSData;
        auto start = tracker.GetPosition();
        auto data = GPSData(1,1,M_PI_2,0);
        /*data.LatVar(8);
        data.LongVar(8);
        data.HeadingVar(8);*/
        testEvent(data);
        auto end = tracker.GetPosition();
        QVERIFY(start != end);
        QVERIFY(end.Latitude - 0.999201 < 0.000001);
        QVERIFY(end.Longitude - 0.999201 < 0.000001);
        QVERIFY(end.Heading - 1.56954 < 0.00001);
        QVERIFY(end.Latitude.Variance - 7.99361 < 0.00001);
        QVERIFY(end.Longitude.Variance - 7.99361 < 0.00001);
        QVERIFY(end.Heading.Variance - 7.99361 < 0.00001);
    }

    void testUpdateFromIMU_NoAccel()
    {
        PositionTracker tracker;
        Event<IMUData> testEvent;
        testEvent += &tracker.LOnIMUData;
        auto start = tracker.GetPosition();
        auto data = IMUData(0,0,0,0,0,0);
        testEvent(data);
        auto end = tracker.GetPosition();
        QCOMPARE(start, end);
    }
};

#endif // TESTPOSITIONTRACKER_HPP
