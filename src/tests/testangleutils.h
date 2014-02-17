#ifndef TESTANGLEUTILS_H
#define TESTANGLEUTILS_H

#include <QtTest>
#include "common/utils/AngleUtils.h"
#include <iostream>
#include <iomanip>
#include <cmath>

class TestAngleUtils: public QObject
{
    Q_OBJECT

    private Q_SLOTS:
    void degToRads()
    {
        using namespace std;
        double angle = 42.0;
        double rads = AngleUtils::degToRads(angle);
        QCOMPARE(rads, 42.0*(M_PI/180.0));

    }
    void radsToDeg()
    {
        using namespace std;
        double rads = 42.0*(M_PI/180.0);
        double angle = AngleUtils::radsToDeg(rads);
        QCOMPARE(angle, 42.0);

    }
};

#endif // TESTANGLEUTILS_H
