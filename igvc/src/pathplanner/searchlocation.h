#ifndef SEARCHLOCATION_H
#define SEARCHLOCATION_H

#include <iostream>

class SearchLocation
{
public:
  float X, Y, Theta;

  static constexpr float sameness_threshold = 0.005;

  SearchLocation()
  {
  }
  SearchLocation(float X, float Y, float Theta);

  bool operator==(const SearchLocation &other) const;
  bool operator<(const SearchLocation &other) const;
  float distTo(SearchLocation other, double resolution) const;

  friend std::ostream &operator<<(std::ostream &stream, const SearchLocation &loc)
  {
    stream << "(" << loc.X << "," << loc.Y << "," << loc.Theta << ")";
    return stream;
  }
};

#endif  // SEARCHLOCATION_H
