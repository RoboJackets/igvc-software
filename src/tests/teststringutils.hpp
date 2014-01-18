#ifndef TESTSTRINGUTILS_H
#define TESTSTRINGUTILS_H

#include <QtTest>
#include <common/utils/StringUtils.hpp>

class TestStringUtils: public QObject
{
    Q_OBJECT

private Q_SLOTS:
    void testCase1()
    {
        using namespace std;

        vector<string> tokens = split("www.robojackets.org", '.');
        QCOMPARE(tokens.size(), 3UL);
        QCOMPARE(tokens[0],string("www"));
        QCOMPARE(tokens[1],string("robojackets"));
        QCOMPARE(tokens[2],string("org"));
    }
};

#endif
