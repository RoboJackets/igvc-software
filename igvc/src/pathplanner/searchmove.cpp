#include "searchmove.h"

SearchMove::SearchMove(double v, double w, double dt)
{
  V = v;
  W = w;
  DeltaT = dt;
}

bool SearchMove::operator==(const SearchMove &other)
{
  return (V == other.V) && (W == other.W) && (DeltaT == other.DeltaT);
}
