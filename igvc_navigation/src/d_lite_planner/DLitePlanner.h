/**
D* lite path planner implementation.

D* lite is an incremental search algorithm that keeps track of data from previous
searches to speed up replanning. The first search through the grid space is
equivalent to A*, with nodes in the priority queue ordered by a path cost estimate:
    f(s) = g(s) + h(s_start, s)

When the graph is updated with new sensor information and the path needs replanning,
comparatively few nodes need expanding to re-calculate the optimal path.

The DLitePlanner interfaces with the Graph object to calculate the optimal
path through the occupancy grid in an eight-connected grid space. This means
that each node has 8 neighbors and can only travel to those eight neighbors.

Author: Alejandro Escontrela <aescontrela3@gatech.edu>
Date Created: December 22nd, 2018

Sources:
D* Lite [Sven Koenig, Maxim Likhachev]
http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf

Optimal and Efficient Path Planning for Unknown and Dynamic Environments  (D*)
[Anthony Stentz]
https://pdfs.semanticscholar.org/77e9/b970024bc5da2b726491823f7d617a303811.pdf

MIT Advanced Lecture 1: Incremental Path Planning
[MIT OCW]
https://ocw.mit.edu/courses/aeronautics-and-astronautics/16-412j-cognitive-robotics-spring-2016/videos-for-advanced-lectures/advanced-lecture-1/
*/

#ifndef DLITEPLANNER_H
#define DLITEPLANNER_H

#include "Graph.h"
#include "Node.h"
#include "PriorityQueue.h"

#include <cmath>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

class DLitePlanner
{
public:
  // Graph contains methods to deal with Node(s) as well as updated occupancy
  // grid cells
  Graph node_grid_;

  std::vector<std::tuple<int, int>> path_;

  float goal_dist_;

  /**
  Sets value for GoalDist, the minumum value from the goal node a node must
  be before the search is considered complete.

  @param[in] goalDist the minimum distance from the goal
  */
  void setGoalDistance(float goal_dist);

  /**
  Calculate the key for a node S.

  key defined as <f1(s), f2(s)>
  where...
  f1(s) = min(g(s), rhs(s)) + h(s_start, s) + KeyModifier
  f2(s)min(g(s), rhs(s))

  @param[in] s Node to calculate key for
  @return calculated key
  */
  Key calculateKey(Node s);
  /**
  Initializes the graph search problem by setting g and rhs values for start
  node equal to infinity. For goal node, sets g value to infinity and rhs value
  to 0. Inserts goal node into priority queue to initialize graph search problem.
  */
  void initializeSearch();
  /**
  Clears the previous search's contents and re-initializes the search problem.
  Clears the Node cache (expanded_map_), the priority queue, and all cell updates that
  occured in the previous timestep.
  */
  void reInitializeSearch();
  /**
  Updates a node's standing in the graph search problem. Update dependant upon
  the node's g value and rhs value relative to each other.

  Locally inconsistent nodes (g != rhs) are inserted into the priority queue
  while locally consistent nodes are not.

  @param[in] s Node to update
  */
  void updateNode(Node s);
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
  Constructs the optimal path through the search space by greedily choosing
  the next node that minimizes c(s,s') + g(s'). Ties are broken arbitrarily.
  */
  void constructOptimalPath();
  /**
  Tries to insert an entry into the unordered map. If an entry for that key
  (Node) already exists, overrides the value with specified g and rhs vals.

  @param[in] s key to create entry for
  @param[in] g g value for entry
  @param[in] rhs rhs value for entry
  */
  void insertOrAssign(Node s, float g, float rhs);
  /**
  Checks whether a specified node is within range of the goal node. This 'range'
  is specified by the GOAL_RANGE instance variable.

  @param[in] s Node to check
  @return whether or not node s is within range of the goal
  */
  bool isWithinRangeOfGoal(Node s);
  /**
  Returns g-value for a node s

  @param[in] s Node to get g-value for
  @return g-value
  */
  float getG(Node s);
  /**
  Returns rhs value for a node s

  @param[in] s Node to get rhs-value for
  @return rhs-value
  */
  float getRHS(Node s);
  /**
  Returns list of indices of expanded nodes, that is, the indices of the
  nodes contained in the unordered map.

  @return vector of nodes in the unordered map
  */
  std::vector<std::tuple<int, int>> getExplored();

private:
  // hashed map contains all nodes and <g,rhs> values in search
  std::unordered_map<Node, std::tuple<float, float>> expanded_map_;
  // priority queue contains all locally inconsistent nodes whose values
  // need updating
  PriorityQueue priority_queue_;
};

#endif
