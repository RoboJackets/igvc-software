/**
Represents a vertex on the graph (corner on the occupancy grid)

Author: Alejandro Escontrela <aescontrela3@gatech.edu>
Date: Dec. 15th, 2018
*/

#ifndef NODE_H
#define NODE_H

#include <vector>
#include <tuple>
#include "Cell.h"

class Node
{
public:
    /**
    Constructors for the node
    */
    Node(float x, float y, std::tuple<Cell,Cell,Cell,Cell> cells);
    Node(float x, float y, std::tuple<Cell,Cell,Cell,Cell> cells, float g, float rhs);

    /**
    Get the tuple representing the (x,y) coordinates of this node

    @return std::tuple representing node's cartesian coordinates
    */
    std::tuple<float,float> getCoords();
    /**
    Get the cells surrounding this node

    @return cells surrounding this node
    */
    std::tuple<Cell,Cell,Cell,Cell> getCells();
    /**
    Gets the path cost estimate for the current node. This path cost, or g-value,
    represents the cost of a path from the node to the goal.

    @return the node's g-value
    */
    float getCost();
    /**
    Gets the rhs-values of the node. RHS-values are one-step lookahead values
    based on the g-values.

    @return rhs-value of node
    */
    float getRHS();
    /**
    Set the g-value for the current node

    @param[in] val g-balue to be set
    */
    void setCost(float val);
    /**
    Set the RHS value for the current node

    @param[in] val rhs value to be set
    */
    void setRHS(float val);



private:
    float g;
    float rhs;
    float x;
    float y;

    /**
    Each node is surrounded by up to 4 cells. Cells indexed as follows:
    -> cells[0] -> top right
    -> cells[1] -> top left
    -> cells[2] -> bottom left
    -> cells[3] -> bottom right
    */
    std::tuple<Cell,Cell,Cell,Cell> cells;
    std::tuple<float,float> ind;
};


#endif // NODE_H
