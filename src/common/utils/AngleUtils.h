#ifndef ANGLEUTILS_H
#define ANGLEUTILS_H


#include <cmath>

class AngleUtils
{
    public:
    inline static double degToRads(double angle)
    {
        return angle*(M_PI/180.0);
    }

    inline static double radsToDeg(double rad)
    {
        return rad*(180.0/M_PI);
    }
};

#endif // ANGLEUTILS_H
