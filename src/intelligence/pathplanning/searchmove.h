#ifndef SEARCHMOVE_H
#define SEARCHMOVE_H

#include <iostream>

class SearchMove
{
public:
    double V, W;

    SearchMove() {}

    SearchMove(double v, double w);

    bool operator == (const SearchMove &other);

    friend std::ostream &operator << (std::ostream &stream, SearchMove &move)
    {
        stream << "(<" << move.V << "," << move.W << ">";
        return stream;
    }
};

#endif // SEARCHMOVE_H
