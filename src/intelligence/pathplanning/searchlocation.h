#ifndef SEARCHLOCATION_H
#define SEARCHLOCATION_H

#include <iostream>

class SearchLocation
{
public:
    double x, y, theta;

    static constexpr double sameness_threshold = 0.01;

    SearchLocation() { }
    SearchLocation(double _x, double _y, double _theta);

    bool operator == (const SearchLocation &other);
    bool operator < (const SearchLocation &other);
    double distTo(SearchLocation other);

    friend std::ostream &operator<< (std::ostream &stream, SearchLocation &loc)
    {
        stream << "(" << loc.x << "," << loc.y << "," << loc.theta << ")";
        return stream;
    }
};

#endif // SEARCHLOCATION_H
