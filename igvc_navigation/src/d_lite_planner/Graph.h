/**
The graph contains the nodes and cells that set the foundation of the search
problem.In Field D*, Nodes are located on the corners of grid cells (as opposed to the
center, like in A*, D*, or D*lite).

The Graph object provides an interface for the occupancy grid, as well as nodes.

Author: Alejandro Escontrela <aescontrela3@gatech.edu>
Date Created: December 16, 2018
*/

#ifndef GRAPH_H
#define GRAPH_H

#include "igvc_utils/NodeUtils.hpp"
#include "Node.h"

#include <igvc_msgs/map.h>

#include <cv_bridge/cv_bridge.h>
#include <utility>
#include <vector>
#include <tuple>
#include <cmath>
#include <limits>

class Graph
{
public:
    /**
    Prev map is the occupancy grid one timestep ago, it's used to check for changes in the
    the occupancy grid. Map is the current, most up-to-date occupancy grid.
    */
    cv_bridge::CvImagePtr Map;

    Node Start; // start node in the search problem
    Node Goal; // goal node in the search problem

    // Updated cell information is used to update nodes that lie on the 4 corners of
    // each updated cell. This is reset each time updateGraph is called. Each element
    // is composed of <cell index, old value, new value>
    std::vector<std::tuple<int,int>> updatedCells;

    // dimensions of the occupancy grid (number of cells)
    int length;
    int width;

    float Resolution; // grid resolution
    float CSpace; // configuration space

    float DIAGONAL_COST = (float) sqrt(2);
    float EDGE_COST = 1;

    // k_m, as defined in the D* lite paper, keeps track of the robot's movement
    // in the grid space and increased new node's key vaues by k_m as to maintain
    // lower bounds
    float K_M = 0;

    /**
    Default no-arg constructor
    */
    Graph();
    ~Graph();

    /**
    Sets a value for the graph's configuration space

    @param[in] CSpace a value for the configuration space
    */
    void setCSpace(float CSpace);
    /**
    Sets the goal node for the Field D* search problem

    @param[in] std::tuple containing x,y index of goal
    */
    void setGoal(std::tuple<int,int> Goal);
    /**
    Loads the parameters for the occupancy grid

    @param[in] msg map to load parameters from
    */
    void initializeGraph(const igvc_msgs::mapConstPtr& msg);
    /**
    Loads a new occupancy grid into the graph object. This is used to update
    all nodes and vertices with the updated path costs.

    @param[in] map cv ptr containing a mono8-encoded version of the occupancy grid
    */
    void updateGraph(igvc_msgs::mapConstPtr& msg);
    /**
    Determines whether or not a Node is valid. The only condition that would
    render a node invalid is if it falls out of the grid's boundary.

    @param[in] s Node for which to check validity
    @return whether or not the node is valid
    */
    bool isValidNode(Node s);
    /**
    Determines whether or not a cell is valid based off of its index. As with
    the Node, the only condition that would render a node invalid is if it is
    out of bounds
    */
    bool isValidCell(std::tuple<int,int> ind);
    /**
    Determines whether or not s_prime is a diagonal neighbor to s under the
    assumption that s_prime is a neighbor to begin with.

    @param[in] s reference node
    @param[in] s_prime possible diagonal node

    @return whether or not s_prime is diagonal to s
    */
    bool isDiagonal(Node s, Node s_prime);
    /**
    Returns neighbors of node s on an eight-grid layout. That is, on average,
    each node s has 8 neighbors.

    @return vector containing the 8 neighbors of node s. When there is no neighbord
            (as is the case on corners or along edges), a null value is returned
    */

    std::vector<Node> nbrs(Node s, bool include_invalid = false);
    /**
    Returns first counter-clockwise neighbor of node s and a neighbor node
    s', starting at s'.

    @param[in] s_prime a node neighboring this object node.
    @return the first counter-clockwise neighbor node  of s and s'
    */
    Node ccknbr(Node s, Node s_prime);
    /**
    Returns first clockwise neighbor of node s and a neighbor node
    s', starting at s'.

    @param[in] s_prime a node neighboring this object node.
    @return the first clockwise neighbor node  of s and s'
    */
    Node cknbr(Node s, Node s_prime);
    /**
    Returns a vector of consecutive neighbor tuples. A pair of consecutive
    neighbors is defined as two neighbors of s that are joined along an edge.
    By convention, consecutive neighbords will be returned in a clockwise order.

    *s4   *s3   *s2 => In this case connbrs(s) = {(s1,s2),(s2,s3),(s3,s4),
    *s5   *s    *s1                               (s4,s5),(s5,s6),(s6,s7),
    *s6   *s7   *s8                               (s7,s8),(s8,s1)}

    If a pair of consecutive neighbors contains an invalid node, the pair is
    omitted. The backpointer of a node s, therefore, is the most clockwise
    node in the consecutive neighbor pair (the first index of each pair).
    */
    std::vector<std::tuple<Node, Node>> connbrs(Node s);
    /**
    Returns traversal cost of node s and a diagonal node s'. If cell or any of its
    surrounding CSpace is occupied, infinity is returned.

    @param[in] s reference node
    @param[in] s_prime diagonal node

    @return diagonal traversal cost
    */
    float getC(Node s, Node s_prime);
    /**
    Returns traversal cost of node s and s', a non-diaginal (vertical or
    horizontal) neighbor of s. Cost taken to be the maximum cost of
    two cells neighboring the edge.

    @param[in] s reference node
    @param[in] s_prime diagonal node

    @return edge traversal cost
    */
    float getB(Node s, Node s_prime);
    /**
    Gets cost of traversing the grid cell while taking configuration space
    into account

    @param[in] ind index of cell to calculate cspace-corrected cost for
    @return cost of traversing grid cell with cspace
    */
    float getValWithCSpace(std::tuple<int,int> ind);
    /**
    Gets nodes affected by updated cell value while taking into account CSpace

    @return list of reference nodes whose values may have been affected by
    the updated cell cost
    */
    std::vector<Node> getUpdatedCellNodesWithCSpace();
    /**
    Get cost of traversing from a Node s to a neighboring node s_prime

    @param[in] s node to get traversal cost through
    @param[in] s_prime neighbor of s
    */
    float getTraversalCost(Node s, Node s_prime);
    /**
    Calculates euclidian distance between the start node 'Start' and the
    specified node s. This will be used as the focussing heuristic.

    @param[in] s node to calculate euclidian distance to
    @return euclidian distance between the start node and node s
    */
    float euclidian_heuristic(Node s);
    /**
    Same as above method but takes index of Node to calculate euclidian distance
    from start for.

    @param[in] ind (x,y) coordinates of node
    @return euclidian distance to node from current start position
    */
    float euclidian_heuristic(std::tuple<int,int> ind);
    /**
    Gets Nodes around a cell whose occupancy value has changed while taking configuration
    space into account.

    @param[in] cellInd index of cell whose val has changed
    @return list of nodes who might be affected by changed cell value
    */
    std::vector<Node> getNodesAroundCellWithCSpace(std::tuple<int,int> cellInd);



private:


};

#endif // GRAPHSEARCH_H
