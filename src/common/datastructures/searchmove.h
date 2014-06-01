#ifndef SEARCHMOVE_H
#define SEARCHMOVE_H

#include <iostream>

class SearchMove
{
public:
    double V, W, DeltaT;

    SearchMove() {}

    SearchMove(double v, double w, double dt);

    bool operator == (const SearchMove &other);

    friend std::ostream &operator << (std::ostream &stream, SearchMove &move)
    {
        stream << "(<" << move.V << "," << move.W << ">";
        return stream;
    }
};

#endif // SEARCHMOVE_H
