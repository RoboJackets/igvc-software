#ifndef SEARCHMOVE_H
#define SEARCHMOVE_H

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

  friend std::ostream &operator<<(std::ostream &stream, SearchMove &move)
  {
    stream << "(<" << move.X << "," << move.Y << ">";
    return stream;
  }
};

#endif  // SEARCHMOVE_H
