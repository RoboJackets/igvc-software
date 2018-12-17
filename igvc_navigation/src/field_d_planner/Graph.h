/**
The graph contains the nodes and cells that set the foundation of the search
problem.
*/

#ifndef GRAPH_H
#define GRAPH_H

#include "igvc_utils/NodeUtils.hpp"
#include <vector>
#include <tuple>

class Graph
{
public:

    /**
    #TODO: what's the input argument to this constructor?
        - what graph will be used
    */
    Graph();
    ~Graph();

    /**
    Returns neighbors of node s on an eight-grid layout. That is, on average,
    each node s has 8 neighbors.

    @return vector containing the 8 neighbors of node s. When there is no neighbord
            (as is the case on corners or along edges), a null value is returned
    */
    std::vector<Node> nbrs(Node s);
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
    Compute cost of node s wrt the edge formed by s_prime and s_pp, two
    consecutive neighbors of s.
    */
    float computeCost(Node s, Node s_prime, Node s_pp);
    /**
    Computes the key of a node s

    @param[in] s Node to compute key for
    @return key in the form of a tuple [f-value; g-value]
    */
    std::tuple<float, float> key(Node s);



private:


};

#endif // GRAPHSEARCH_H
