#include "Node.h"
#include <igvc_utils/NodeUtils.hpp>

Node::Node(bool valid)
{
  this->valid = valid;
}

Node::Node(int x, int y)
{
  this->x_ = x;
  this->y_ = y;
  this->ind_ = std::make_tuple(x, y);
}
Node::Node(std::tuple<int, int> ind) : Node(std::get<0>(ind), std::get<1>(ind))
{
}

Node::~Node()
{
}

void Node::setIndex(int x, int y)
{
  this->x_ = x;
  this->y_ = y;
  this->ind_ = std::make_tuple(x, y);
}

void Node::setIndex(std::tuple<int, int> ind)
{
  this->setIndex(std::get<0>(ind), std::get<1>(ind));
}

std::tuple<int, int> Node::getIndex() const
{
  return this->ind_;
}

void Node::setBptr(std::tuple<int, int> bptr)
{
  this->bptr_ = bptr;
}

std::tuple<int, int> Node::getBptr() const
{
  return this->bptr_;
}

float Node::distTo(std::tuple<float, float> position)
{
  return igvc::get_distance(static_cast<std::tuple<float, float>>(this->getIndex()), position);
}

bool Node::operator==(const Node& other) const
{
  return this->getIndex() == other.getIndex();
}

bool Node::operator!=(const Node& other) const
{
  return !(*this == other);
}

Node& Node::operator=(const Node& node)
{
  // do the copy
  std::tie(this->x_, this->y_) = node.getIndex();
  this->ind_ = node.getIndex();
  this->bptr_ = node.getBptr();

  // return the existing object so we can chain this operator
  return *this;
}

std::ostream& operator<<(std::ostream& stream, const Node& n)
{
  int x, y;
  std::tie(x, y) = n.getIndex();
  stream << "[" << x << ", " << y << "]";
  return stream;
}

std::ostream& operator<<(std::ostream& stream, const std::unordered_set<Node>& uset)
{
  stream << "{";
  for (auto const& i : uset)
  {
    stream << i << ", ";
  }
  stream << "}";

  return stream;
}
