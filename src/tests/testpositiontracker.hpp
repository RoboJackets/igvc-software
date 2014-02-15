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

    void testUpdateFromIMU_AccelX()
    {
        PositionTracker tracker;
        Event<IMUData> testEvent;
        testEvent += &tracker.LOnIMUData;
        testEvent(IMUData(0,0,0,0,0,0));
        auto start = tracker.GetPosition();

        double time = 2.0;
        double accel = 1.0;
        double d = (0.0*time) + (0.5 * accel * time * time);

        usleep(2000000);
        auto data = IMUData(0,0,0,accel,0,0);
        testEvent(data);
        auto end = tracker.GetPosition();
        QVERIFY(start != end);
        QVERIFY(end.Heading == start.Heading);
        QVERIFY(end.Longitude == start.Longitude);
        QVERIFY(end.Latitude != start.Latitude);

        std::cout << "Heading: "<<end.Heading << std::endl;
        std::cout << "Latitude: "<<end.Latitude << std::endl;
        std::cout << "Longitude: "<<end.Longitude << std::endl;

    }

    void testUpdateFromIMU_AccelY()
    {
        PositionTracker tracker;
        Event<IMUData> testEvent;
        testEvent += &tracker.LOnIMUData;
        testEvent(IMUData(0,0,0,0,0,0));
        auto start = tracker.GetPosition();

        double time = 2.0;
        double accel = 1.0;
        double d = (0.0*time) + (0.5 * accel * time * time);

        usleep(2000000);
        auto data = IMUData(0,0,0,0,accel,0);
        testEvent(data);
        usleep(2000000);
        auto data2 = IMUData(0,0,0,0,0.5,0);
        testEvent(data2);
        auto end = tracker.GetPosition();
        QVERIFY(start != end);
        //QVERIFY(end.Heading == start.Heading);
        //QVERIFY(end.Longitude == start.Longitude);
        //QVERIFY(end.Latitude != start.Latitude);
        std::cout << "Heading: "<<end.Heading << std::endl;
        std::cout << "Latitude: "<<end.Latitude << std::endl;
        std::cout << "Longitude: "<<end.Longitude << std::endl;
    }
};

#endif // TESTPOSITIONTRACKER_HPP
