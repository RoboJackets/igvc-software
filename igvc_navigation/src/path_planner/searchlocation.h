#ifndef SEARCHLOCATION_H
#define SEARCHLOCATION_H

#include <iostream>
#include <list>

class SearchLocation
{
public:
  std::list<double> PrevTheta;
  double X, Y, Theta, ThetaChange;
  double cost;

  static constexpr double sameness_threshold = 0.005;

  SearchLocation()
  {
  }
  SearchLocation(double X, double Y, double Theta);

  bool operator==(const SearchLocation &other) const;
  bool operator<(const SearchLocation &other) const;
  double distTo(const SearchLocation &other, double resolution) const;

  friend std::ostream &operator<<(std::ostream &stream, const SearchLocation &loc)
  {
    stream << "(" << loc.X << "," << loc.Y << "," << loc.Theta << ")";
    return stream;
  }
};

#endif  // SEARCHLOCATION_H
