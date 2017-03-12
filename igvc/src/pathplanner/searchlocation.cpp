#include "searchlocation.h"
#include <math.h>  // pow() & sqrt()
#include <cmath>   // abs()

SearchLocation::SearchLocation(float _x, float _y, float _theta)
{
  x = _x;
  y = _y;
  theta = _theta;
}

bool SearchLocation::operator==(const SearchLocation &other) const
{
  return std::abs(x - other.x) < sameness_threshold &&
         std::abs(y - other.y) < sameness_threshold;  // && abs(theta - other.theta) < sameness_threshold;
}

bool SearchLocation::operator<(const SearchLocation &other) const
{
  if ((*this) == other)
    return false;
  else if (x != other.x)
  {
    return x < other.x;
  }
  else if (y != other.y)
  {
    return y < other.y;
  }
  else if (theta != other.theta)
  {
    return theta < other.theta;
  }
  else
  {
    return false;
  }
}

float SearchLocation::distTo(SearchLocation other) const
{
  return sqrt(pow(other.x - x, 2) + pow(other.y - y, 2));
}
