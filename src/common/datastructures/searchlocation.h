#ifndef SEARCHLOCATION_H
#define SEARCHLOCATION_H

#include <iostream>

class SearchLocation
{
public:
    float x, y, theta;

    static constexpr float sameness_threshold = 0.1;

    SearchLocation() { }
    SearchLocation(float _x, float _y, float _theta);

    bool operator == (const SearchLocation &other);
    bool operator < (const SearchLocation &other) const;
    float distTo(SearchLocation other);

    friend std::ostream &operator<< (std::ostream &stream, SearchLocation &loc)
    {
        stream << "(" << loc.x << "," << loc.y << "," << loc.theta << ")";
        return stream;
    }
};

#endif // SEARCHLOCATION_H
