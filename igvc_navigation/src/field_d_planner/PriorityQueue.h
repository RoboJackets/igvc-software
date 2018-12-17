/**
Priority queue to hold all locally inconsistent nodes to be expanded in the
graph search problem. Inherits std::priority_queue.

Author: Alejandro Escontrela <aescontrela3@gatech.edu>
Date Created: Dec. 15th, 2018
*/

#include <queue>
#include <tuple>
#include <vector>
#include <string>
#include <sstream>
#include "Node.h"
#include "NodeCompare.hpp"


class PriorityQueue : public std::priority_queue<Node, std::vector<Node>, NodeCompare>
{

public:

    /**
    Remove Node from priority queue. O(n)

    @param[in] s Node to remove
    @return whether or not Node was found
    */
    bool remove(Node s);

    /**
    Return string representation of the items in the priority queue. O(n)

    @return string of items in the priority queue
    */
    std::string toString();

    /**
    Update item sharing the same key as the specified item in the priority queue
    O(n)

    @param[in] Item to update in priority queue
    @return whether or not item was successfully updated
    */
    bool update(Node n);

    /**
    Check if priority_queue contains item with key. O(n)

    @param[in] key string to check for
    @return whether or not the priority queue contains that item
    */
    bool contains(Node n);

};
