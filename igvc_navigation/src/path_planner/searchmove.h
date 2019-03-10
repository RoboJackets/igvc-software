#ifndef SEARCHMOVE_H
#define SEARCHMOVE_H

/*
 * Defines an action for the search problem
 * X,Y are the changes that are anticipated
 */

#include <iostream>
#include <limits>

class SearchMove
{
public:
  double X, Y;

  SearchMove()
  {
  }

  SearchMove(double x, double y);

  bool operator==(const SearchMove &other);

  friend std::ostream &operator<<(std::ostream &stream, const SearchMove &move)
  {
    stream << "(<" << move.X << "," << move.Y << ">)";
    return stream;
  }
};

#endif  // SEARCHMOVE_H
