#ifndef TIMING_H
#define TIMING_H

#include "boost/date_time/local_time/local_time.hpp"

/***
Function that returns time since epoch in milliseconds using the boost libraries.
Inlined because I'm lazy and because it will probably boost efficiency since it will be called so often.
***/
inline long long  milliseconds_since_epoch()
{
    using boost::gregorian::date;
    using boost::posix_time::ptime;
    using boost::posix_time::microsec_clock;

    static ptime const epoch(date(1970, 1, 1));
    long long ms = (microsec_clock::universal_time() - epoch).total_milliseconds(); //value is too large to fit in 4 bits. long long used to help force 8bit size
    return ms;
}

#endif // TIMING_H
