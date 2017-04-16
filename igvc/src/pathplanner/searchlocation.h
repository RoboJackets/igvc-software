#ifndef SEARCHLOCATION_H
#define SEARCHLOCATION_H

#include <iostream>

class SearchLocation
{
public:
  float x, y, theta;

  static constexpr float sameness_threshold = 0.005;

  SearchLocation()
  {
  }
  SearchLocation(float _x, float _y, float _theta);

  bool operator==(const SearchLocation &other) const;
  bool operator<(const SearchLocation &other) const;
  float distTo(SearchLocation other) const;

  friend std::ostream &operator<<(std::ostream &stream, const SearchLocation &loc)
  {
    stream << "(" << loc.x << "," << loc.y << "," << loc.theta << ")";
    return stream;
  }
};

#endif  // SEARCHLOCATION_H
