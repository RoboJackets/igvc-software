#ifndef TIMING_H
#define TIMING_H

#include "boost/date_time/local_time/local_time.hpp"

/***
Function that returns time in seconds using the boost libraries.
Inlined because I'm lazy and because it will probably boost efficiency since it will be called so often.
***/
inline double  seconds_since_IGVCpoch()
{
    using boost::gregorian::date;
    using boost::posix_time::ptime;
    using boost::posix_time::microsec_clock;
    double millisPerSecond = 1000.0;
    static ptime const epoch(date(2013, 2, 2));
    double theTime = (microsec_clock::universal_time() - epoch).total_milliseconds()/millisPerSecond;
    //std::cout<<theTime<< std::endl;
    return theTime;
}

#endif // TIMING_H
