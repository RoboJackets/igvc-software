#include "searchlocation.h"
#include <math.h> // pow() & sqrt()
#include <stdlib.h> // abs()

SearchLocation::SearchLocation(double _x, double _y, double _theta)
{
    x = _x;
    y = _y;
    theta = _theta;
}

bool SearchLocation::operator == (const SearchLocation &other)
{
    return abs(x - other.x) < sameness_threshold && abs(y - other.y) < sameness_threshold;// && abs(theta - other.theta) < sameness_threshold;
}

bool SearchLocation::operator < (const SearchLocation &other)
{
    if(x < other.x)
    {
        return true;
    }
    else if(y < other.y)
    {
        return true;
    }
    else if(theta < other.theta)
    {
        return true;
    }
    return false;
}

double SearchLocation::distTo(SearchLocation other)
{
    return sqrt(pow(other.x - x, 2) + pow(other.y - y, 2));
}
