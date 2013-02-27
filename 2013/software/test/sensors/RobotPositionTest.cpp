#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE Module1

#include <boost/test/unit_test.hpp>
#include "sensors/timing.h"
#include "sensors/DataStructures/SensorData.h"
#include "sensors/DataStructures/DataArray.h"
#include "sensors/timing.h"
#include "sensors/DataStructures/GPSData.h"
#include "sensors/RobotPosition.h"
#include "sensors/DataStructures/DataPoint.hpp"




using namespace boost::unit_test;

GPSAccuracy GPSData::NAV200Default = GPSAccuracy(.0001, .0001, 3, 0.01);
GPSAccuracy IMUData::ArduPilotDefault= GPSAccuracy(.001, .001, 1, .1);

BOOST_AUTO_TEST_CASE(testUpdate1)
{
    double currTime = seconds_since_IGVCpoch();

    RobotPosition Mistii;
    Mistii.update(GPSData(49.12, 0.2122, 11,1, currTime));
    BOOST_CHECK_EQUAL(Mistii.Lat()[0].value(), 49.12);

    //BOOST_CHECK(Mistii.Long()[0], 49.12);
    //BOOST_CHECK(Mistii.Speed()[0], 49.12);
    //BOOST_CHECK(Mistii.Heading()[0], 49.12);
}

