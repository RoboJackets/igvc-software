/**
Field D* Incremental Path Planner Implementation

Field D* is an incremental search algorithm that keeps track of data from previous
searches to speed up replanning. The first search through the grid space is
equivalent to A*, with nodes in the priority queue ordered by a path cost estimate:

    f(s) = g(s) + h(s_start, s) + key_modifier

When the graph is updated with new sensor information and the path needs replanning,
comparatively few nodes need expanding to re-calculate the optimal path.

Unlike D* Lite, the Field D* Path Planning algorithm is an "any-angle path planner",
meaning it can compute global paths that are not restricted to a specific heading
increment. As such, The Field D* path planning algorithm can generate smooth paths
around high-cost regions of the cost map.

FieldDPlanner interfaces with the Graph object to calculate the optimal
path through the occupancy grid in an eight-connected grid space. This means
that each node has 8 neighbors.

Author: Alejandro Escontrela <aescontrela3@gatech.edu>
Date Created: December 22nd, 2018

Sources:
Field D* [Dave Ferguson, Anthony Stentz]
https://pdfs.semanticscholar.org/58f3/bc8c12ee8df30b3e9564fdd071e729408653.pdf

MIT Advanced Lecture 1: Incremental Path Planning
[MIT OCW]
https://ocw.mit.edu/courses/aeronautics-and-astronautics/16-412j-cognitive-robotics-spring-2016/videos-for-advanced-lectures/advanced-lecture-1/
*/

#ifndef FIELDDPLANNER_H
#define FIELDDPLANNER_H

#include "Graph.h"
#include "Node.h"
#include "PriorityQueue.h"

#include <assert.h>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

/**
The CostComputation struct contains the linearly interpolated path cost as
computed by the computeCost method. Additionally, it stores the resulting x and
y traversal distances calculated during the traversal cost computation.
*/
struct CostComputation
{
  float x;
  float y;
  float cost;

  CostComputation(float x, float y, float cost)
  {
    this->x = x;
    this->y = y;
    this->cost = cost;
  }

  // overloaded assignment operator
  CostComputation& operator=(const CostComputation& other)
  {
    this->x = other.x;
    this->y = other.y;
    this->cost = other.cost;

    return *this;
  }
};

class FieldDPlanner
{
public:
  // Graph contains methods to deal with Node(s) as well as updated occupancy
  // grid cells
  Graph NodeGrid;

  std::vector<Position> Path;

  // path additions made by one step of constructOptimalPath()
  typedef std::pair<std::vector<Position>, float> path_additions;

  float GoalDist;

  /**
  Sets value for GoalDist, the minumum value from the goal node a node must
  be before the search is considered complete.

  @param[in] goalDist the minimum distance from the goal
  */
  void setGoalDistance(float goalDist);

  /**
  Compute the linearly interpolated path cost of three neighboring positions p,
  p_a, and p_b. The resulting path cost of p is dependent on the relative cost of
  the two consecutive neighbors (p_a, p_b) as well as the traversal cost to
  both neighbors (c, b).

  @param[in] p continuous position on the graph to calculate the path cost for
  @param[in] p_a consecutive neighbor of p
  @param[in] p_b another consecutive neighbor of p
  @return a tuple containing the path cost of p and resulting (x,y) traversal distances
          (relative to p_a and p_b) of the path cost calculation
  */
  CostComputation computeCost(const Position& p, const Position& p_a, const Position& p_b);
  CostComputation computeCost(const Node& s, const Node& s_a, const Node& s_b);
  /**
  Returns true if position p is a valid vertex on the graph. A position is a
  valid vertex if both of its cartesian coordinates are integers and it lies within
  the bounds of the graph

  @param[in] p position to assess
  @return whether or not p is a vertex
  */
  bool isVertex(const Position& p);
  /**
  Returns the linearly interpolated path cost of a lying along an edge. Such an edge
  node is characterized by having one of its cartesian coordinate values a float.
  i.e. (x,y) = (5, 7.48). A vertex node (both coordinates integers) simply have
  their true path cost returned.

  @param[in] p positon on the graph to obtain path cost for
  @return path cost of continuous position
  */
  float getEdgePositionCost(const Position& p);

  /**
  Calculate the key for a node S.

  key defined as <f1(s), f2(s)>
  where...
  f1(s) = min(g(s), rhs(s)) + h(s_start, s) + K_M
  f2(s)min(g(s), rhs(s))

  @param[in] s Node to calculate key for
  @return calculated key
  */
  Key calculateKey(const Node& s);
  /**
  Initializes the graph search problem by setting g and rhs values for start
  node equal to infinity. For goal node, sets g value to infinity and rhs value
  to 0. Inserts goal node into priority queue to initialize graph search problem.
  */
  void initializeSearch();
  /**
  Updates a node's standing in the graph search problem. Update dependant upon
  the node's g value and rhs value relative to each other.

  Locally inconsistent nodes (g != rhs) are inserted into the priority queue
  while locally consistent nodes are not.

  @param[in] s Node to update
  */
  void updateNode(const Node& s);
  /**
  Expands nodes in the priority queue until optimal path to goal node has been
  found. The first search is equivalent to an A* heuristic search. All calls
  to computeShortestPath() thereafter only expand the nodes necessary to
  compute the optimal path to the goal node.

  @return number of nodes expanded in graph search
  */
  int computeShortestPath();
  /**
  Updates nodes around cells whose occupancy values have changed. Takes into
  account the cspace of the robot. This step is performed after the robot
  moves and the occupancy grid is updated with new sensor information

  @return the number of nodes updated
  */
  int updateNodesAroundUpdatedCells();
  /**
  Constructs the optimal path through the search space by greedily moving to
  the next position (or vertex) that minimizes the linearly interpolated
  path cost calculation. Constructed paths populate the `path` attribute

  @param[in] lookahead_dist number of future cell traversals to recurisively
          analyze before moving to next position
  */
  void constructOptimalPath(int lookahead_dist);
  /**
  Helper method for path reconstruction process. Finds the next path position(s)
  when planning from a vertex or an edge position on the graph given the current
  position p and its consecutive neighbors p_a, p_b.

  @param[in] p edge on graph to plan from
  @param[in] p_a consecutive neighbor of p
  @param[in] p_b consecutive neighbor of p
  @return vector containing the next positions(s) and movement cost
  */
  path_additions computeOptimalCellTraversal(const Position& p, const Position& p_a, const Position& p_b);
  /**
  Helper method for path reconstruction process. Finds the next path position(s)
  when planning from a vertex or an edge position on the graph.

  @param[in] p edge on graph to plan from
  @return vector containing the next positions(s) and movement cost
  */
  path_additions getPathAdditions(const Position& p, int lookahead_dist);
  /**
  Checks whether a specified node is within range of the goal node. This 'range'
  is specified by the GOAL_RANGE instance variable.

  @param[in] s Node to check
  @return whether or not node s is within range of the goal
  */
  bool isWithinRangeOfGoal(const Position& p);
  /**
  Tries to insert an entry into the unordered map. If an entry for that key
  (Node) already exists, overrides the value with specified g and rhs vals.

  @param[in] s key to create entry for
  @param[in] g g value for entry
  @param[in] rhs rhs value for entry
  */
  void insert_or_assign(Node s, float g, float rhs);
  /**
  Returns g-value for a node s

  @param[in] s Node to get g-value for
  @return g-value
  */
  float getG(const Node& s);
  /**
  Returns rhs value for a node s

  @param[in] s Node to get rhs-value for
  @return rhs-value
  */
  float getRHS(const Node& s);
  /**
  Returns list of indices of expanded nodes, that is, the indices of the
  nodes contained in the unordered map.

  @return vector of nodes in the unordered map
  */
  std::vector<std::tuple<int, int>> getExplored();

private:
  // hashed map contains all nodes and <g,rhs> values in search
  std::unordered_map<Node, std::tuple<float, float>> umap;
  // priority queue contains all locally inconsistent nodes whose values
  // need updating
  PriorityQueue PQ;
};

#endif
