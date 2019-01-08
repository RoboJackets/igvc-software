/**
Field D* incremental path planner implementation

Field D* is an incremental search algorithm utilizes past search information to
perform fast replanning. Like with D* Lite, the initial search through the grid
space is equivalent to A*, with nodes in the priority queue ordered by a path cost estimate:
    f(s) = g(s) + h(s_start, s)

When the graph is updated with new sensor information and the path needs replanning,
comparatively few nodes need expanding to re-calculate the optimal path.

Field D* extends D* Lite by allowing travel not only to the eight surrounding
neigbors, but also to the continuous space along the edge formed by two consecutive
neighbors. This allows for the generation of smoother, more optimal paths through
the grid space.

Author: Alejandro Escontrela <aescontrela3@gatech.edu>
Date Created: January 1st, 2019 {Happy New Year! └╏ ･ ᗜ ･ ╏┐ }

Sources:
The Field D* Algorithm for Improved Path Planning and
Replanning in Uniform and Non-Uniform Cost Environments
[Dave Ferguson, Anthony Stentz]
https://pdfs.semanticscholar.org/58f3/bc8c12ee8df30b3e9564fdd071e729408653.pdf

D* Lite
[Sven Koenig, Maxim Likhachev]
http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf

Optimal and Efficient Path Planning for Unknown and Dynamic Environments  (D*)
[Anthony Stentz]
https://pdfs.semanticscholar.org/77e9/b970024bc5da2b726491823f7d617a303811.pdf

MIT Advanced Lecture 1: Incremental Path Planning
[MIT OCW]
https://ocw.mit.edu/courses/aeronautics-and-astronautics/16-412j-cognitive-robotics-spring-2016/videos-for-advanced-lectures/advanced-lecture-1/
*/


#ifndef FIELDDPLANNER_H
#define FIELDDPLANNER_H

#include "igvc_navigation/Graph.h"
#include "igvc_navigation/Node.h"
#include "igvc_navigation/PriorityQueue.h"

#include <unordered_map>
#include <utility>
#include <vector>
#include <limits>
#include <cmath>
#include <math.h>

struct ContinuousNeighbors
{
    ContinuousNeighbors() {}
    ContinuousNeighbors(std::tuple<float,float> s_a, std::tuple<float,float> s_b, float cost_s_a, float cost_s_b)
    {
        this->s_a = s_a;
        this->s_b = s_b;
        this->cost_s_a = cost_s_a;
        this->cost_s_b = cost_s_b;
    }

    void setValues(std::tuple<float,float> s_a, std::tuple<float,float> s_b, float cost_s_a, float cost_s_b)
    {
        this->s_a = s_a;
        this->s_b = s_b;
        this->cost_s_a = cost_s_a;
        this->cost_s_b = cost_s_b;
    }
    std::tuple<float,float> s_a; // position of continuous neighbor 1
    std::tuple<float,float> s_b; // position of continuous neighbor 2

    float cost_s_a; // path cost of s_a
    float cost_s_b; // path cost of s_b
};

class FieldDPlanner
{
public:
    // Graph contains methods to deal with Node(s) as well as updated occupancy
    // grid cells
    Graph graph;

    std::vector<std::tuple<float,float>> path;
    typedef std::pair<std::vector<std::tuple<float,float>>,float> path_additions;

    float GOAL_DIST = 0.95f;

    FieldDPlanner();
    ~FieldDPlanner();

    /**
    Calculate the key for a node s. The key is used for organizing nodes in the
    priority queue.

    key defined as <f1(s), f2(s)>
    where...
        f1(s) = min(g(s), rhs(s)) + h(s_start, s) + K_M
        f2(s) = min(g(s), rhs(s))

    @param[in] s Node to calculate key for
    @return calculated key
    */
    Key calculateKey(Node s);
    /**
    Computes minimum path cost of s given any two consecutive neigbors s_a and
    s_b. X and Y values dependant upon the traversal cost to the diagonal node and
    the vertical/horizontal node as well as the relative g-values of both nodes.

    @param[in] s Node to calculate cost for
    @param[in] s_a consecutive neighbor #1 of s
    @param[in] s_b consecutive neighbor #2 of s
    @return path cost of node s
    */
    float computeCost(Node s, Node s_a, Node s_b);
    /**
    Initializes the graph search problem by setting g and rhs values for start
    node equal to infinity. For goal node, sets g value to infinity and rhs value
    to 0. Inserts goal node into priority queue to initialize graph search problem.
    */
    void initialize();
    /**
    Clears the previous search's contents and re-initializes the search problem.
    Clears the Node cache (umap), the priority queue, and all cell updates that
    occured in the previous timestep.
    */
    void reinitialize();
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
    TODO:make private
    returns true if position p is a vertex on the graph. Alternatively, returns
    false if it's an edge.

    @param[in] p position to check
    @return whether or not p is a vertex
    */
    bool isVertex(std::tuple<float,float> p);
    /**
    TODO: make private
    Helper method for path reconstruction process. Finds the next path
    position(s) when planning is performed from a vertex on the graph.

    @param[in] p position on graph to plan from
    @return vector containing next position(s) and movement cost
    */
    path_additions getNextPositionsFromVertex(std::tuple<float,float> p);
    /**
    TODO: make private
    Helper method for path reconstruction process. Finds the next path position(s)
    when planning from an edge on the graph.

    @param[in] p edge on graph to plan from
    @return vector containing the next positions(s) and movement cost
    */
    path_additions getNextPositionsFromEdge(std::tuple<float,float> p);
    /**
    TODO: make private
    TODO: is there a better way of doing this?
    Gathers the continuous neighbors of an edge position. This involves the
    creation of two "imaginary nodes", which are not actually nodes but instead
    lie along an edge on the graph. The path cost of an imaginary node is calculated
    by linearly interpolating the path costs of the two nearby nodes.

                            s5        s4
                            #* * * *#* * * *# s3
                            *       *       *
                        in1 X       X s     X in2
                            *       *       *
                            #* * * *#* * * *#
                          s6        s1      s2

    Here, s is the position whose consectutive neighbors we are trying to gather.
    in1 and in2 are two imaginary nodes whose path costs were calculated through
    the linear interpolation of s2,s3 and s5,s6, respectively.

    @param[in] p position to gather nearest neighbors for.
    @return vector of ContinuousNeighbors objects for each valid consecutive neighbors pair found
    */
    std::vector<ContinuousNeighbors> getContinuousNeighbors(std::tuple<float,float> p);
    /**
    TODO: make private
    Returns true if ContinuousNeighbors object contains valid neighbors (don't
    lie outside of graph).

    @param[in] cn ContinuousNeighbors object to check
    @return whether ContinuousNeighbors is valid or invalid
    */
    bool isValidContinuousNeighbors(ContinuousNeighbors CN);
    /**
    Tries to insert an entry into the unordered map. If an entry for that key
    (Node) already exists, overrides the value with specified g and rhs vals.

    @param[in] s key to create entry for
    @param[in] g g value for entry
    @param[in] rhs rhs value for entry
    */
    void insert_or_assign(Node s, float g, float rhs);
    /**
    TODO: make private
    Checks whether a specified node is within range of the goal node. This 'range'
    is specified by the GOAL_RANGE instance variable.

    @param[in] s Node to check
    @return whether or not node s is within range of the goal
    */
    bool isWithinRangeOfGoal(std::tuple<float,float> p);
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
    std::vector<std::tuple<int,int>> getExplored();
private:
    // this hashed map contains key-value pairs mapping from all
    // expanded nodes to their corresponding g and rhs values
    std::unordered_map<Node,std::tuple<float,float>> umap;
    // priority queue contains all locally inconsistent nodes whose values
    // need updating
    PriorityQueue PQ;

};

std::ostream& operator<<(std::ostream& stream, const ContinuousNeighbors& CN);

#endif
