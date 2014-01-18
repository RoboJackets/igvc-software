#include <QtTest>
#include <common/utils/StringUtils.hpp>
#include <iostream>

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


QTEST_APPLESS_MAIN(TestStringUtils)

#include "teststringutils.moc"
