#include "Node.h"

Node::Node(float x, float y, std::tuple<Cell,Cell,Cell,Cell> cells)
{
    this->x = x;
    this->y = y;
    this->cells = cells;

    this->ind = std::make_tuple(x,y);
}

Node::Node(float x, float y, std::tuple<Cell,Cell,Cell,Cell> cells, float g, float rhs)
{
    this->x = x;
    this->y = y;
    this->cells = cells;
    this->g = g;
    this->rhs = rhs;

    this->ind = std::make_tuple(x,y);
}

std::tuple<float,float> Node::getCoords()
{
    return this->ind;
}

std::tuple<Cell,Cell,Cell,Cell> Node::getCells()
{
    return this->cells;
}

float Node::getCost()
{
    return this->g;
}

float Node::getRHS()
{
    return this->rhs;
}

void Node::setCost(float val)
{
    this->g = val;
}

void Node::setRHS(float val)
{
    this->rhs = val;
}
