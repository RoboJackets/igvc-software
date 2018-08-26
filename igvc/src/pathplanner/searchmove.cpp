#include "searchmove.h"

SearchMove::SearchMove(double x, double y)
{
  X = x;
  Y = y;
}

bool SearchMove::operator==(const SearchMove &other)
{
  return (X == other.X) && (Y == other.Y);
}
