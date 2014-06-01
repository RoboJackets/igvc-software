#ifndef PATHTYPE_HPP
#define PATHTYPE_HPP

#include <vector>

#include <common/datastructures/searchlocation.h>
#include <common/datastructures/searchmove.h>

typedef std::vector<std::pair<SearchMove, SearchLocation>> path_t;

#endif // PATHTYPE_HPP
