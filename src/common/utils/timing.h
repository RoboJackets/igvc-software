#ifndef TIMING_H
#define TIMING_H

#include <boost/date_time/local_time/local_time.hpp>

/***
Function that returns time in seconds using the boost libraries.
Inlined because I'm lazy and because it will probably boost efficiency since it will be called so often.
***/
inline double  micro_seconds_since_IGVCpoch()
{
    using boost::gregorian::date;
    using boost::posix_time::ptime;
    using boost::posix_time::microsec_clock;
    static ptime const epoch(date(2013, 2, 2));
    long long int theTime = (microsec_clock::universal_time() - epoch).total_microseconds();
    return theTime;
}

#endif // TIMING_H
