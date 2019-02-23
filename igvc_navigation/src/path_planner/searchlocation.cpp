#include "searchlocation.h"
#include <math.h>  // pow() & sqrt()
#include <cmath>   // abs()

bool SearchLocation::operator==(const SearchLocation &other) const
{
  return X == other.X && Y == other.Y && Theta == other.Theta;
}

bool SearchLocation::operator<(const SearchLocation &other) const
{
  if ((*this) == other)
    return false;
  else if (X != other.X)
  {
    return X < other.X;
  }
  else if (Y != other.Y)
  {
    return Y < other.Y;
  }
  else if (Theta != other.Theta)
  {
    return Theta < other.Theta;
  }
  else
  {
    return false;
  }
}

double SearchLocation::distTo(const SearchLocation &other, double resolution) const
{
  return sqrt(pow(other.X - X, 2) + pow(other.Y - Y, 2)) * resolution;
}
