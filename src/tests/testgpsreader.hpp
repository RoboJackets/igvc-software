#ifndef TESTGPSREADER_H
#define TESTGPSREADER_H

#include <QtTest>
#include <common/datastructures/GPSData.hpp>
#include <common/logger/logger.h>
#include <common/utils/gpsfilereader.h>
#include <iostream>


/**
 * @brief Tests the GPSFileReader
 *
 * @note Error text in the log is perfectly normal during these tests.
 */
class TestGPSReader: public QObject
{
    Q_OBJECT

private Q_SLOTS:
    void testProperFormat()
    {
        std::queue<GPSData> q;
        try {
        GPSFileReader::read("../test_data/GPSFileReader-testData/Successful/properlyFormedGPSData.txt", q);
        } catch (...) {
            QFAIL("Reader failed to load properly formatted file.");
        }
        double trueLats[] { 33.7871928, 33.7871928, 33.7871932, 33.7871933, 33.7871937, 33.7871938, 33.7871938 };
        double trueLongs[] { -84.4063265, -84.4063265, -84.4063263, -84.4063262, -84.4063262, -84.406326, -84.406326 };
        for(int i = 0; i < 7; i++)
        {
            GPSData data = q.front();
            QCOMPARE(data.Lat(), trueLats[i]);
            QCOMPARE(data.Long(), trueLongs[i]);
        }
    }

    void testTooFewFields()
    {
        try {
            std::queue<GPSData> q;
            GPSFileReader::read("../test_data/GPSFileReader-testData/Unsuccessful/tooFewFields.txt", q);
        } catch(GPSFileFormatException) {
            // Test successful!
            return;
        }
        QFAIL("GPSFileFormatException was not thrown.");
    }

    void testTooManyFields()
    {
        try {
            std::queue<GPSData> q;
            GPSFileReader::read("../test_data/GPSFileReader-testData/Unsuccessful/tooManyFields.txt", q);
        } catch(GPSFileFormatException) {
            // Test successful!
            return;
        }
        QFAIL("GPSFileFormatException was not thrown.");
    }

    void testImproperValue()
    {
        try {
            std::queue<GPSData> q;
            GPSFileReader::read("../test_data/GPSFileReader-testData/Unsuccessful/improperValue.txt", q);
        } catch(GPSFileFormatException) {
            // Test successful!
            return;
        }
        QFAIL("GPSFileFormatException was not thrown.");
    }
};

#endif
