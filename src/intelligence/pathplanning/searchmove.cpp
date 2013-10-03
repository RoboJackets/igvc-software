#include "searchmove.h"

SearchMove::SearchMove(double v, double w)
{
    V=v;
    W=w;
}

bool SearchMove::operator ==(const SearchMove &other)
{
    return (V == other.V) && (W == other.W);
}
