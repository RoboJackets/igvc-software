/**
The Node object represents a vertex on the graph.

In Field D*, Nodes are located on the corners of grid cells (as opposed to the
center, like in A*, D*, or D*lite). The Node is principle currency of Field D*
since, with Nodes, we can define a cost field from which we obtain an
optimal path.

Author: Alejandro Escontrela <aescontrela3@gatech.edu>
Date: Dec. 15th, 2018
*/

#ifndef NODE_H
#define NODE_H

#include <functional>
#include <iostream>
#include <limits>
#include <tuple>
#include <unordered_set>

class Node
{
public:
  /**
  Default constructor. Valid boolean used for returning invalid nodes (usually
  nodes that are out of range) from methods that must return a node object.
  */
  Node(bool valid = true);

  /**
  Takes as parameters the index in the graph that
  this node belongs to.

  @param[in] x x-component of occupancy grid index
  @param[in] y y-component of occupancy grid index
  */
  Node(int x, int y);
  /**
  Same as above constructor except that it received a tuple of (x,y) coords.

  @param[in] ind tuple of x,y coords
  */
  Node(std::tuple<int, int> ind);
  /**
  Destructor
  */
  ~Node();
  /**
  Set the coordinates of a Node

  @param[in] x x index of node
  @param[in] y y index of node
  */
  void setIndex(int x, int y);
  /**
  Set the coordinates of a Node

  @param[in] ind tuple of integer values containing the x and y index
  */
  void setIndex(std::tuple<int, int> ind);
  /**
  Get the tuple representing the (x,y) index of this node

  @return std::tuple representing node's cartesian coordinates
  */
  std::tuple<int, int> getIndex() const;
  /**
  When updating neighbors, we only consider the states actually affected by
  the new value of the popped state and how those states are affected. To do
  this, we keep track of a backpointer for each state, specifying from which
  states it currently derives its path cost. Since, in Field D*, the successor
  of each state is a point on an edge connecting two of its neighboring states,
  this backpointer needs to specify the two states that form the endpoints of
  this edge. bptr(Node s) refers to the _most clockwise_ of the two endpoint
  states relative to state s.

  @param[in] s_prime backpointer index of the current node
  */
  void setBptr(std::tuple<int, int> bptr);
  /**
  Gets the backpointer of the current state s

  @return index of backpointer of current state
  */
  std::tuple<int, int> getBptr() const;

  /**
  equals operator for Node.

  Two nodes are equal if their indices are equal, regardless of g values and
  rhs values.

  @param[in] other Node to compare against
  */
  bool operator==(const Node& other) const;
  /**
  not equals operator for Node.

  @param[in] other Node to compare against
  */
  bool operator!=(const Node& other) const;

  /**
  Overloaded assignment operator

  @param[in] node node whose value to copy
  @return assignment
  */
  Node& operator=(const Node& node);

  bool valid = true;

private:
  int x;
  int y;

  std::tuple<int, int> ind;
  std::tuple<int, int> bptr;
};

// ostream operator for Node
std::ostream& operator<<(std::ostream& stream, const Node& n);

// ostream operator for std::unordered_set of Node
std::ostream& operator<<(std::ostream& stream, const std::unordered_set<Node>& uset);

// Re-defined hash method for nodes. Used for maintaining unordered map.
namespace std
{
template <>
struct hash<Node>
{
  std::size_t operator()(const Node& node) const
  {
    std::size_t result = 17;

    int x, y;
    std::tie(x, y) = node.getIndex();

    result = 31 * result + x;
    result = 31 * result + y;
    return result;
  }
};
}  // namespace std

#endif  // NODE_H
