#ifndef SEARCHLOCATION_H
#define SEARCHLOCATION_H

/*
 * Class used to define state for the search problem
 * X,Y,Theta define the state of the vehicle
 * PrevTheta is a list of previous theta changes
 * ThetaChange is the current accumulation of the theta changes
 * cost is the current path cost at that node
 */

#include <iostream>
#include <list>

class SearchLocation
{
public:
  std::list<double> PrevTheta;
  double X, Y, Theta, ThetaChange;
  double cost;

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
