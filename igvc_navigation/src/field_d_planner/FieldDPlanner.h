/**
This implementation of Field D* builds on D* Lite to allow for the traversal
of a grid through nodes other than the eight neighbors on an eight-directed
graph. Being an incremental search algorithm, node g-values are stored from
search to search, and updated when edge costs change.

The first iteration of Field D* (the first call to computeShortestPath())
mirrors A* exactly. The benefit of incremental search is that all calls to
computeShortestPath() thereafter must expand relatively few nodes, allowing for
real-time path planning.

The two main components of the Field D* implementation are:
1. Priority queue (PQ) of inconsistent states. This PQ holds all states
whose estimated values D* must update to make them consistent with their
neighbors.
2. Unordered Set/HashSet US. US holds all of the expanded nodes in the graph
search problem. In turn, US is used to hold most up-to-date g and rhs values
and to construct the optimal path from the start node to the end node. As a
benefit, node lookup can be performed in O(1) time, thus helping achieve the
goal of real-time path planning.

Author: Alejandro Escontrela  <aescontrela3@gatech.edu>
Date Created: December 21st, 2018

Field D* [Dave Ferguson, Anthony Stentz]
https://pdfs.semanticscholar.org/58f3/bc8c12ee8df30b3e9564fdd071e729408653.pdf

D* Lite [Sven Koenig, Maxim Likhachev]
http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf

Optimal and Efficient Path Planning for Unknown and Dynamic Environments  (D*)
[Anthony Stentz]
https://pdfs.semanticscholar.org/77e9/b970024bc5da2b726491823f7d617a303811.pdf
*/
#ifndef FIELDDPLANNER_H
#define FIELDDPLANNER_H

#include "Graph.h"
#include "Node.h"
#include "PriorityQueue.h"

#include <unordered_set>
#include <limits>

class FieldDPlanner
{
public:
    // Graph contains methods to deal with Node(s) as well as updated occupancy
    // grid cells
    Graph graph;

    /**
    Computes the key of a node s

    @param[in] s Node to compute key for
    @return key in the form of a tuple [f-value; g-value]
    */
    std::tuple<float, float> key(Node s);
    /**
    Compute cost of node s wrt the edge formed by s_prime and s_pp, two
    consecutive neighbors of s.
    */
    float computeCost(Node s, Node s_prime, Node s_pp);
    /*
    Initializes the Field D* graph search problem by setting the start g and
    rhs value to infinity, the goal g value to inifinity and rhs value to 0, and
    inserting the goal state into the priority queue.
    */
    void initialize();
    /**
    Obtains proper values for a node with the specified reference node. If a
    node has already been initialized and is in the hashed set, then get its
    g-values and rhs-values. Otherwise, get values of node with g = rhs = inf
    and indices of ref_ind.

    @param[in] to_get reference to a Node whose value we need to obtain
    @param[in] ref_ind Node whose index will be used to sit first parametere's values
    */
    void getNode(Node& to_get, Node ref_node);
    /**
    Updates a state in the priority queue and hash set based on estimated
    cost of neighbors.

    @param[in] s Node to update
    */
    void updateState(Node s);
    /**
    Updates inconsistent nodes in priority queue until all relevant nodes are
    locally consistent and a shortest path can be created. Once relevant nodes
    in the state space are consistent, optimal path can be constructed by
    beginning at the start node and greedily moving towards the next best point
    along a neighboring edge.
    */
    void computeShortestPath();

private:
    // hashed set contains all nodes in search
    std::unordered_set<Node> US;
    // priority queue contains all locally inconsistent nodes whose values
    // need updating
    PriorityQueue PQ;
};

#endif
