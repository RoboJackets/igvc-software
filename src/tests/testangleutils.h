#ifndef TESTANGLEUTILS_H
#define TESTANGLEUTILS_H

#endif // TESTANGLEUTILS_H
#include <QtTest>
#include "common/utils/AngleUtils.h"
#include <iostream>
#include <iomanip>

class TestAngleUtils: public QObject
{
    Q_OBJECT

    private Q_SLOTS:
    void testCase1()
    {
        using namespace std;
        double angle = 42.0;
        double rads = AngleUtils::angleToRads(angle);
        std::cout<<rads<<endl;
        double angle2 = AngleUtils::radsToAngle(rads);
        std::cout<<angle2<<endl;
    }
};
